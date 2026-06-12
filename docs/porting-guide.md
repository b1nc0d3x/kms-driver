# Porting a hardware driver to kms

This guide covers what a hardware DRM driver must provide to plug into
kms and light up `/dev/dri/cardN` on its own silicon.  It assumes
familiarity with FreeBSD `newbus` and standard driver patterns
(`device_attach`, `bus_space_map`, `OFW_BUS_ATTACH`).

The framework owns: cdev lifecycle, ioctl dispatch, mode-config root,
object ID allocator, dumb-buffer allocator + mmap, DRM event queue,
EDID/DP-AUX/mode helpers, vblank delivery primitive.

The driver owns: MMIO mapping, clock + power-domain setup, KMS topology
(CRTC / plane / encoder / connector), modeset register programming,
framebuffer scanout, vblank source, hot-plug detection (where
applicable).

`sys/dev/kms/stub/kms_stub.c` is the minimum-viable example
(no real hardware).  `sys/dev/kms/drivers/rk_kms.c` is the
full reference (RK3399 VOP_BIG + DW HDMI + Cadence MHDP DP).

## 1. Module skeleton

A driver is a kernel module that talks to a real device via newbus and
registers a `struct drm_device` with the framework.

```c
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_mode.h>
#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_encoder.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_modes.h>
#include <kms/drm_plane.h>
#include <kms/drm_vblank.h>

MALLOC_DECLARE(M_KMS);

struct myhw_softc {
	device_t		 dev;
	struct drm_device	*drm_dev;
	struct drm_crtc		 crtc;
	struct drm_plane	 primary;
	struct drm_encoder	 encoder;
	struct drm_connector	 connector;
	/* MMIO handles, clocks, resets, sysctls live here. */
};

static const struct drm_driver myhw_drm_driver = {
	.name		= "myhw",
	.desc		= "MyVendor display",
	.date		= "20260611",
	.major		= 1,
	.minor		= 0,
	.patchlevel	= 0,
	.driver_features = 0,
};
```

A driver does **not** define its own `M_KMS`.  Sharing the
framework's malloc type avoids a uma-zone collision during kldload.

## 2. KMS topology

Every driver wires the same five-object graph:

```
    crtc <-- primary plane
      ^
      |
   encoder
      |
   connector
```

Construct it in `device_attach` after MMIO is mapped but before the
device is registered:

```c
static int
myhw_topology_init(struct myhw_softc *sc)
{
	struct drm_mode_config *mc = &sc->drm_dev->mode_config;
	uint32_t crtc_mask;
	int error;

	mc->max_width = 4096;
	mc->max_height = 4096;

	error = kms_crtc_init(sc->drm_dev, &sc->crtc, &myhw_crtc_funcs);
	if (error != 0)
		return (error);
	crtc_mask = 1u << sc->crtc.index;

	error = kms_plane_init(sc->drm_dev, &sc->primary,
	    &myhw_plane_funcs, DRM_PLANE_TYPE_PRIMARY, crtc_mask,
	    myhw_primary_formats, nitems(myhw_primary_formats));
	if (error != 0)
		return (error);
	sc->crtc.primary_plane = &sc->primary;

	error = kms_encoder_init(sc->drm_dev, &sc->encoder,
	    &myhw_encoder_funcs, DRM_MODE_ENCODER_TMDS);
	if (error != 0)
		return (error);
	sc->encoder.possible_crtcs = crtc_mask;

	error = kms_connector_init(sc->drm_dev, &sc->connector,
	    &myhw_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	if (error != 0)
		return (error);
	kms_connector_attach_encoder(&sc->connector, &sc->encoder);

	return (0);
}
```

`crtc_mask` propagates through plane and encoder so the framework can
match the userspace SETCRTC request to the right pipe.  For a
single-pipe driver `crtc_mask` is always 1.

## 3. The five driver hooks

These four function tables are the framework's only entry points into
the driver.  All are optional; missing hooks are no-ops with sensible
default behavior.

### `struct drm_crtc_funcs`

| hook          | called from        | purpose                          |
|---------------|--------------------|----------------------------------|
| `set_config`  | `DRM_IOCTL_MODE_SETCRTC` | program the pipe (mode, fb)  |
| `page_flip`   | `DRM_IOCTL_MODE_PAGE_FLIP` | swap scanout fb, emit event   |
| `destroy`     | unregister         | per-CRTC cleanup                 |

`set_config` is where modeset register programming lives.  It receives
a `struct drm_mode_set` with the requested mode (NULL = blank) and fb
(NULL = no scanout).

### `struct drm_plane_funcs`

| hook       | purpose                              |
|------------|--------------------------------------|
| `destroy`  | per-plane cleanup                    |

Primary plane scanout state today lives entirely on the CRTC; the plane
object is bookkeeping for the framework's plane-aware ioctls.

### `struct drm_encoder_funcs`

| hook       | purpose                              |
|------------|--------------------------------------|
| `destroy`  | per-encoder cleanup                  |

### `struct drm_connector_funcs`

| hook         | purpose                                          |
|--------------|--------------------------------------------------|
| `get_modes`  | populate `connector->modes` (EDID or canned)     |
| `destroy`    | per-connector cleanup                            |

If the sink advertises EDID, fetch it via your DDC or DP-AUX I2C engine
then hand the bytes to `kms_edid_add_modes(connector, edid)`.
For sinks that lie about EDID or fixed panels with no DDC, build a
`struct drm_display_mode` via `kms_mode_create()` and call
`kms_connector_add_mode(connector, mode)`.

## 4. Device registration

After topology is wired, register the card so the framework creates
`/dev/dri/cardN`:

```c
error = kms_dev_register(&myhw_drm_driver, sc, &sc->drm_dev);
if (error != 0)
	return (error);
```

The `sc` pointer is stashed as `drm_dev->driver_priv`; recover it in
hook callbacks via `crtc->dev->driver_priv`.

`kms_dev_register` walks past any in-use minor so kms
coexists with `drm2` on the same host.

Mirror with `kms_dev_unregister(sc->drm_dev)` in `device_detach`,
after `*_cleanup` calls for every KMS object.

## 5. Framebuffer scanout

When userspace allocates a dumb buffer via `DRM_IOCTL_MODE_CREATE_DUMB`,
the framework allocates contiguous guest RAM pages and stores them in
the GEM object's `pages[]` array.  When SETCRTC delivers a `set->fb`,
walk to the underlying physical address like this:

```c
static vm_paddr_t
myhw_fb_paddr(struct drm_framebuffer *fb)
{
	struct drm_gem_object *obj;

	if (fb == NULL)
		return (0);
	obj = fb->gem_objs[0];
	if (obj == NULL || obj->pages == NULL || obj->npages == 0)
		return (0);
	return (VM_PAGE_TO_PHYS(obj->pages[0]));
}
```

`fb->pitches[0]` carries the row stride that ADDFB2 declared, and
`fb->width`/`fb->height` mirror the mode.  Plumb these into your
scanout-engine registers.

If your scanout engine has a sub-4-GiB DMA window (RK3399's VOP_BIG
WIN0_YRGB_MST is one example), validate the PA before programming it
and return `EINVAL` from `set_config` if it's out of reach.

## 6. Vblank events

Two paths flow through vblank delivery:

1. `DRM_IOCTL_WAIT_VBLANK` tsleeps on the CRTC address and is woken
   from `kms_vblank_handler`.
2. `DRM_IOCTL_MODE_PAGE_FLIP` with `DRM_MODE_PAGE_FLIP_EVENT` stashes
   `pending_flip_file` + `pending_flip_user_data` on the CRTC; the next
   vblank delivers a `DRM_EVENT_FLIP_COMPLETE` and clears the stash.

The driver chooses where vblank ticks come from.  Options:

- **Hardware IRQ.**  Wire the VOP-end-of-frame IRQ to a fast handler,
  call `kms_vblank_handler(&sc->crtc)` from the ithread.
- **Software ticker.**  If the SoC's IRQ wiring is fragile or out of
  scope for the first phase, run a `timeout_task` on
  `taskqueue_thread` at the mode's vrefresh interval and call
  `kms_vblank_handler` from the task.  See
  `rk_kms_vblank_task` for the pattern.  Wall-clock-accurate
  enough for SLiM/Xorg event flow; not for tear-free composition.

The handler call is the same in both cases:

```c
void kms_vblank_handler(struct drm_crtc *crtc);
```

It advances `crtc->sequence`, wakes WAIT_VBLANK sleepers, and delivers
any pending PAGE_FLIP_EVENT.

## 7. Public API summary

| Function | Purpose |
|---|---|
| `kms_dev_register / _unregister` | card lifecycle |
| `kms_crtc_init / _cleanup` | CRTC object |
| `kms_plane_init / _cleanup` | plane object |
| `kms_encoder_init / _cleanup` | encoder object |
| `kms_connector_init / _cleanup / _attach_encoder` | connector graph |
| `kms_connector_add_mode / _modes_clear` | mode list mgmt |
| `kms_mode_create / _destroy / _set_name / _vrefresh` | display-mode helpers |
| `kms_edid_parse / _add_modes / _checksum` | EDID helpers |
| `kms_dp_aux_init / _transfer / _dpcd_read / _dpcd_write` | DP-AUX helpers |
| `kms_gem_object_create / _get / _put / _release_all` | GEM lifecycle |
| `kms_gem_handle_create / _delete / _lookup / _object_lookup_offset` | GEM handle table |
| `kms_send_event / _send_vblank_event` | event queue delivery |
| `kms_vblank_handler` | vblank tick entrypoint |
| `kms_mode_object_find / _register / _unregister / _put` | KMS object lookup |
| `kms_framebuffer_init / _cleanup` | FB lifecycle |
| `kms_display_mode_to_modeinfo` | uapi/framework mode bridge |

All exported via `EXPORT_SYMS` in `sys/modules/kms/core/Makefile`.
Add `MODULE_DEPEND(myhw, kms, 1, 1, 1)` in the driver module.

## 8. Hardware-side checklist

The framework can't know what your silicon needs before MMIO works.
These steps cover the typical SoC display block.

- [ ] Map every MMIO range your scanout path touches: VOP/CRTC,
      GRF/SYSCON, CRU/clock controller, PMU/power-domain controller,
      HDMI / DP / DSI controller.
- [ ] Bring required power domains up before any register write into
      them.  PMU `PWRDN_CON`/`PWRDN_ST` style polling, then poll until
      ST clears.  Skipping this is the #1 cause of "writes hang the
      AXI bus."
- [ ] Clear any BUS_IDLE_REQ bits that would park the AXI bridge to
      your block.
- [ ] Ungate fabric + DCLK + HCLK + ACLK clocks for the scanout block,
      and any GATEDIS lockout bit that auto-gates them.
- [ ] Set up the pixel-clock PLL (VPLL, BPLL, etc.) for the requested
      mode.  Source the scanout DCLK from it.
- [ ] If u-boot left the controller running for a splash framebuffer,
      reset or re-enter the controller's standby state before
      reprogramming timing — writing live registers under an active
      pixel clock can corrupt mid-frame state.
- [ ] Mirror the per-modeset version of the power/clock sanity step
      from attach.  The boot framebuffer's domain state can drift
      between attach and userspace's first SETCRTC; calling
      domain-sanity once at attach is not enough.

`rk_kms.c` implements all of the above; `rk_kms_display_domain_sanity`
is the reference for the protocol the last bullet points at.

## 9. Hard rules

- **Public exports use the `kms_` prefix.**  Anything that ends
  up in `EXPORT_SYMS` must not collide with a `drm2` symbol of the
  same name — the linker has resolved cross-module references to
  drm2's copy in the past and silently produced a working-looking
  driver that wrote to drm2's state.
- **No `linker_file_lookup_symbol` for cross-module calls.**  Use
  `extern` declarations + `MODULE_DEPEND(yourdriver, otherdriver, ...)`.
- **No LinuxKPI.**  Pure FreeBSD primitives only: `newbus`, `cdev`,
  `sx`, `mtx`, `vm_pager`, `taskqueue`, `callout`, `malloc(9)`,
  `refcount(9)`, `TAILQ`/`STAILQ`.
- **`style(9)` from line 1.**  Tabs to indent, `} else {` on one line,
  no trailing statements after case/if, block comments with `/*` and
  `*/` on their own lines, <=80 cols.
- **Gate first-time live writes behind a sysctl.**  Boot the driver
  with the modeset path off, then flip it on after kldload to bisect
  hangs.  `rk_kms_vop_program_timing`'s stage bitmask is the
  pattern.
- **Don't kldunload a driver whose framebuffer vt is bound to.**  vt's
  softclock can fire into a freed FB VA and panic.  Return `EBUSY`
  from `device_detach` while userspace still holds the scanout.

## 10. Putting it on the build

In `sys/modules/Makefile` add the driver to the SUBDIR list under the
appropriate platform conditional.  In your driver's own Makefile:

```make
.PATH: ${SRCTOP}/sys/dev/kms/drivers

KMOD=	myhw

SRCS=	myhw.c \
	bus_if.h device_if.h ofw_bus_if.h

CFLAGS+=	-I${SRCTOP}/sys/dev/kms/include
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/uapi

.include <bsd.kmod.mk>
```

Loader.conf to bring up the stack:

```
kms_load="YES"
myhw_load="YES"
```

## 11. Bring-up sanity

After kldload, before flipping any modeset sysctl on:

1. `kldstat | grep myhw` — module loaded, no error.
2. `ls -l /dev/dri/card*` — your card appears.
3. `dmesg | grep myhw` — attach message printed, MMIO map confirmed.
4. `drm_probe /dev/dri/card0` (from `sys/dev/kms/tools/`) —
   VERSION + GET_UNIQUE + GET_CAP return clean.
5. Flip your first modeset sysctl bit (single stage), restart slim or
   issue a SETCRTC, watch dmesg.  If it hangs, the per-stage DPRINTFs
   on the framebuffer console name the last MMIO before the freeze.
