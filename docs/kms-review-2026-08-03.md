# kms review — 2026-08-03 (post ~23K-line tree audit)

Third-party review after the last round (H1 d_mmap UAF, H2 cross-fd
BO read, H3 flip-fb release timing, H4 vblank/close race, H5 no
DRM_MASTER gate) all closed.  New findings, ranked by severity.

## High

### H1 (new) — kms_mmap has no per-file access control

**Impact**: mmap_offset is a global monotonic counter starting at 0.
`kms_gem_object_lookup_offset_containing` searches the whole device.
Any process that can open `/dev/dri/cardN` can mmap any other client's
scanout BO by guessing an offset — i.e. read the screen.

Linux gates this with `drm_vma_node_allow`.  `d_mmap` can't reach
`cdevpriv`, so the fix is `d_mmap_single` plus a per-`drm_file`
allowed-offset set.

### H2 (new) — UAF race in kms_vblank_handler

**Impact**: Ready events are detached into a local ready list and
`flip_file` is captured, then `mc->mutex` is dropped before
`kms_send_event()`.  `kms_file_dtor` only scrubs entries still on
`crtc->pending_vblank_events` — anything already moved to ready is
invisible to it, so the file can be freed in that window.  Needs a
refcount on `drm_file`, or delivery under the lock.

### H3 (new) — Modeset ioctls have no master/auth gate

**Impact**: `is_master` is tracked, `SET_MASTER` is priv_checked —
but nothing ever reads it.  `SETCRTC`, `PAGE_FLIP`, `ATOMIC`,
`ADDFB2`, `RMFB`, `SETGAMMA`, `CURSOR`, `OBJ_SETPROPERTY` are all
reachable by any opener.

### H4 (new) — igen_gtt_bind_gem_at truncates u64 softpin address

**Impact**: `first_idx = (uint32_t)(ggtt_byte_addr / PAGE_SIZE)`
aliases high addresses to low indices, and `bind_end = first_idx +
npages` wraps — so a `first_idx` near `0xFFFFFF00` passes the
reservation-overlap check, and `igen_gtt_write(sc, first_idx + i, ...)`
truncates back into range, sailing past the H7 backstop too.  With
`i915_dispatch_enable` on, that's arbitrary GGTT remap from an
unprivileged client.  Do the arithmetic in u64 and reject
`ggtt_byte_addr >= (uint64_t)GTT_ENTRIES * PAGE_SIZE`.

### H5 (new) — GGTT bindings never torn down

**Impact**: Nothing unbinds on `GEM_CLOSE` or file close, so PTEs
keep pointing at pages the kernel has freed and reused.  Combined
with `submit_user_batch`, that's a GPU-visible UAF into arbitrary
physical memory.

### H6 (new) — I915_GEM_CREATE has no size cap

**Impact**: `gc->size` is u64 straight into `kms_gem_object_create`,
which does `malloc(npages * sizeof(ptr), M_WAITOK)` before it ever
tries the contig allocation.  `CREATE_DUMB` has its 256 MiB cap;
this path has nothing.  Worth capping inside
`kms_gem_object_create` itself.

## Medium

### M1 (new) — kqueue events never fire

**Impact**: `selwakeup()` doesn't fan out to `si_note` — FreeBSD
drivers call `KNOTE_LOCKED()` explicitly alongside it.  As written,
EVFILT_READ attaches but nothing wakes it, so epoll-shim compositors
block forever on FLIP_COMPLETE.  That's the exact failure M2 was
meant to close.  (Worth confirming against the tree's
`kern/sys_generic.c`, but fairly confident.)

### M2 (new) — Unbounded allocations from unprivileged clients

**Impact**:
- `pending_vblank_events` (WAIT_VBLANK with a far-future target, one
  malloc per call, no cap).
- Property blobs (1 MiB each, no per-file cap, never freed on close
  — and `DESTROYPROPBLOB` has no ownership check, so one client can
  nuke the compositor's mode blobs).

### M3 (new) — Page-flip event armed after the hardware is programmed

**Impact**: `funcs->page_flip()` runs, then `pending_flip_file` is
set.  A driver whose vblank fires in between drops the event.

### M4 (new) — SETGAMMA doesn't check gamma_size vs crtc->gamma_size

**Impact**: hands unsafe size to driver hook.  The render-node deny
list also misses `SETGAMMA`, `DIRTYFB`, `CURSOR`/`CURSOR2`, and
`OBJ_SETPROPERTY`.

## Low

- `drm_modeset.c:494` — ungated `printf` on every page flip with
  an event, i.e. 60/sec to the console.
- `/dev/input/event*` chmod in `drm_drv.c` reads as `devfs.rules`
  work done in the kernel; revoke hardcodes `0600 root:wheel` rather
  than restoring what was there, and the grant/revoke counter goes
  unbalanced if the tunable flips between them.
- `kms_prime_fo_mmap` doesn't NULL-check `obj->pager`, and it hands
  back the pager directly — so on arm64 an imported dmabuf gets
  different cache attributes than the same BO via the cdev's `d_mmap`
  UNCACHEABLE path.

## Prior-review status (spot-checked while reading this pass)

- PRIME import no longer double-gets — verified.
- GEM error path defers to `drm_gem_pager_dtor` — verified.
- `kms_framebuffer_check_geometry` — u32-input math can't wrap u64,
  correct.
- `kms_file_dtor` scrubs `mode_config` — verified.
- `primary_fb` properly ref-swapped — verified.
