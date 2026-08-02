# KMS Framework

[![build](https://github.com/b1nc0d3x/kms-driver/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/b1nc0d3x/kms-driver/actions/workflows/build.yml)

A native FreeBSD Kernel Mode Setting (KMS) framework — no LinuxKPI, no
`drm2`, no `drm-kmod`, no Linux compatibility shim of any kind.  Pure
FreeBSD primitives all the way down.

The KMS Framework is the substrate that hardware display drivers (the
in-tree `rk_kms` for Rockchip RK3399 + USB-C DP, future `rk_hdmi`,
`amdgpu_kms`, `vmwgfx_kms`, …) plug into so a modern KMS userspace
(Xorg + modesetting DDX, Wayland compositors via libdrm) can run on
FreeBSD without inheriting Linux's KPI surface.

It owns:

- `/dev/dri/cardN` cdev + ioctl plumbing
- mode-object model (CRTC / encoder / connector / plane / framebuffer)
- DRM uapi structs (in `sys/dev/kms/uapi/drm/` — Linux-uapi-shape
  adapted to native FreeBSD types) so existing userspace binaries link
  unchanged
- the framework-side API hardware drivers consume (in
  `sys/dev/kms/include/kms/`)
- legacy and atomic modeset, dumb buffers + `cdev_pager` mmap, event
  ring, vblank API, DP AUX helpers, EDID + mode helpers

Hardware drivers stay small — they fill four hook tables (CRTC funcs,
encoder funcs, connector funcs, plane funcs), program their silicon,
and let the framework handle everything user-facing.

## Layout

```
sys/dev/kms/
  uapi/drm/        userspace ABI (Linux uapi, FreeBSD-adapted)
  include/kms/     framework's kernel API
  core/            framework implementation
  display/         display helpers (EDID, DP, modes)
  gem/             graphics execution manager
  stub/            validation stub driver
  drivers/         hardware drivers
    rk_kms.c       Rockchip RK3399 VOP + USB-C DP
    x86/igen/      Intel (Kabylake-generation) — WIP
sys/modules/kms/
  core/            builds kms.ko
  stub/            builds kms_stub.ko
sys/modules/rk_kms/  builds rk_kms.ko
```

## Build

Framework only (no external deps):

```sh
make kms-only            # builds kms.ko
```

Full build (framework + rk_kms driver):

```sh
make                     # builds kms.ko + rk_kms.ko
```

`rk_kms.ko` depends on an out-of-tree FUSB302 USB-C PD controller
driver at `dev/iicbus/usb/fusb302_var.h` — install that driver
alongside first, or build only `kms-only` on hosts without it (CI
does the latter).

Cross-build to arm64 from an amd64 host (requires FreeBSD src at
`/usr/src` for the kernel headers):

```sh
env MACHINE=arm64 MACHINE_ARCH=aarch64 \
    CC="cc --target=aarch64-unknown-freebsd15.0" \
    make -B
```

Install (uses `DESTDIR`, defaults to `/`):

```sh
make install
```

## Building inside a FreeBSD source tree

The canonical layout works as drop-in:

```
/usr/src/sys/dev/kms       -> kms/sys/dev/kms
/usr/src/sys/modules/kms   -> kms/sys/modules/kms
/usr/src/sys/modules/rk_kms -> kms/sys/modules/rk_kms
```

with `kms` and `rk_kms` added to the SUBDIR list in
`sys/modules/Makefile`.  Then `make buildkernel MODULES_OVERRIDE="kms
rk_kms"` builds the modules via the normal kernel-modules path.

## Design rules

- Pure FreeBSD primitives: `newbus`, `cdev`, `sx`, `mtx`, `vm_pager`,
  `taskqueue`, `callout`, `malloc(9)`, `refcount(9)`, `TAILQ`/`STAILQ`.
- DRM userspace ABI headers are consumed verbatim with FreeBSD type
  adaptations — they define the userspace contract.
- No LinuxKPI; no Linux-shaped translation layer.
- `style(9)` compliance.

## License

BSD-2-Clause — see `LICENSE`.  Every framework and driver source file
carries the `SPDX-License-Identifier: BSD-2-Clause` header.

The DRM userspace ABI headers under `sys/dev/kms/uapi/drm/` (`drm.h`,
`drm_mode.h`, `drm_fourcc.h`, `drm_sarea.h`) are derived from the
Linux kernel DRM subsystem and keep their upstream MIT-style license.
That license is permissive and BSD-compatible; the top of each of
those files carries its original copyright notice.
