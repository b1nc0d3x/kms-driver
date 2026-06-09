# kms

A native FreeBSD KMS framework — no LinuxKPI, pure FreeBSD primitives.

kms is a substrate that hardware DRM drivers (`rk_drm`, `rk_cdn_dp`,
`rk_hdmi`, …) can consume instead of `drm2`, so a modern KMS userspace
(Xorg + modesetting DDX, Wayland compositors via libdrm) can run on
FreeBSD/arm64 without dragging in `drm-kmod`'s Linux compatibility layer.

The DRM userspace ABI lives in `sys/dev/kms/uapi/drm/` — those headers
are the Linux uapi adapted to native FreeBSD types. The framework's own
kernel API lives in `sys/dev/kms/include/kms/`.

## Status

| Phase | Scope                                            | Status |
|-------|--------------------------------------------------|--------|
| 1     | Skeleton + uapi headers                          | done   |
| 2     | Bare cdev + VERSION / GET_UNIQUE / SET_VERSION / GET_CAP | done   |
| 3     | Mode config root + object ID allocator           | done   |
| 4     | KMS object lifecycle (CRTC / plane / encoder / connector / framebuffer) | done |
| 5     | EDID parser + mode helpers + DP AUX              | done   |
| 6     | Dumb buffers + cdev_pager mmap                   | done   |
| 7     | Legacy modeset (SETCRTC / ADDFB / PAGEFLIP)      | done   |
| 8     | Atomic modeset                                   | done   |
| 9     | Port `rk_drm` onto kms                     | -      |

Phase 2 validated live on FreeBSD/arm64 (RockPro64 / `rk3399`).
`drm_probe` against `/dev/dri/card1` returns:

```
opened /dev/dri/card1 fd=3
VERSION probe: driver 0.1.0  name_len=4 date_len=8 desc_len=22
VERSION fill : name="stub" date="20260605" desc="KMS phase-2 stub"
GET_UNIQUE   : unique_len=11
GET_UNIQUE   : unique="kms:1"
SET_VERSION  : di=1.4  dd=0.1
GET_CAP      : cap=0x1 value=0x0
```

Card minor walks past any in-use `/dev/dri/cardN` so kms coexists
with `drm2` on the same host.

## Layout

```
sys/dev/kms/
  uapi/drm/        userspace ABI (Linux uapi, FreeBSD-adapted)
  include/kms/  framework's kernel API
  core/            framework implementation
  display/         display helpers (EDID, DP, modes) [phase 5]
  gem/             graphics execution manager [phase 6+]
  stub/            phase-2 validation stub driver
  tools/           drm_probe userspace smoke test
sys/modules/kms/
  core/            builds kms.ko
  stub/            builds kms_stub.ko
```

## Build

Host build (amd64 or arm64 native, sanity check):

```sh
make
```

Cross-build to arm64 from amd64 host (requires populated FreeBSD cross
toolchain at `/usr/obj/usr/src/arm64.aarch64/tmp` from a prior
`buildkernel`):

```sh
make TARGET=arm64 TARGET_ARCH=aarch64
```

Smoke test:

```sh
make probe
./sys/dev/kms/tools/drm_probe /dev/dri/card0
```

## Building inside a FreeBSD source tree

The canonical layout works as drop-in:

```
/usr/src/sys/dev/kms       -> kms/sys/dev/kms
/usr/src/sys/modules/kms   -> kms/sys/modules/kms
```

with `kms` added to the SUBDIR list in `sys/modules/Makefile`. Then
`make buildkernel MODULES_OVERRIDE=kms` builds the modules via the
normal kernel-modules path.

## Design rules

- Pure FreeBSD primitives: `newbus`, `cdev`, `sx`, `mtx`, `vm_pager`,
  `taskqueue`, `callout`, `malloc(9)`, `refcount(9)`, `TAILQ`/`STAILQ`.
- DRM userspace ABI headers are consumed verbatim with FreeBSD type
  adaptations — they define the userspace contract.
- No LinuxKPI; no Linux-shaped translation layer.
- `style(9)` compliance from day 1.

## License

BSD-2-Clause. See individual source files for SPDX identifiers.
