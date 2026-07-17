# Boot ordering for igen + sddm on FreeBSD

Getting `sddm_enable="YES"` to work fast at boot on igen requires four
coordinated fixes.  Verified live on fbsdx86 (Dell OptiPlex 5040, Intel
HD 630 Kabylake) 2026-07-17.

## 1. rc.d/igen_attach handles kldload before LOGIN

Ship `conf/rc.d/igen_attach` at `/etc/rc.d/igen_attach` and set
`sysrc igen_attach_enable=YES`.  It does `devctl detach vgapci0 →
kldload kms → kldload igen → devctl rescan pci0`, then fires
`sysctl dev.igen.0.re.gen9_full_bringup_now=1` and refreshes the
XDG_RUNTIME_DIR ownership.

`BEFORE: LOGIN` guarantees sddm's rc.d start finds `/dev/dri/card0`
registered and the display lit.

## 2. Do NOT load igen from /boot/loader.conf

Loader-time kldload wedges the box on Kabylake platforms (physical
recovery required).  Use rc.d/igen_attach instead.

## 3. Patch sddm port's `pgrep getty` wait

`/usr/local/etc/rc.d/sddm`'s `sddm_start()` waits up to 60s for
`pgrep -f "^/usr/libexec/getty "` — but init(8) doesn't spawn gettys
until AFTER `/etc/rc` completes, so the wait always times out.  Replace
`while ! pgrep …; do` with `while false; do` so the loop exits
immediately.  Backup goes OUTSIDE the rc.d directory (rc.d treats every
executable file inside as a service and will fire it too, spawning
duplicate daemons).

## 4. Autologin XDG_RUNTIME_DIR

sddm autologin's PAM refuses the session unless `/var/run/xdg/admin`
is owned by the target user (uid 1001).  rc.d/igen_attach chowns it on
every boot (`/var/run` is tmpfs, ownership doesn't persist).  Autologin
config goes in `/usr/local/etc/sddm.conf`:

```ini
[Autologin]
User=admin
Session=plasma.desktop
Relogin=false
```

## Timing

| Fix stage                              | sshd up | notes                        |
|----------------------------------------|---------|------------------------------|
| Original rc.local (kldload + restart)  | 192 s   | 60 s pgrep wait, 60 s detach |
| Stop-sddm-first ordering               |  55 s   | avoids fd-refcount blocker   |
| sddm_enable=NO + direct-launch         |  42 s   | bypasses rc.d entirely       |
| **rc.d/igen_attach + sddm_enable=YES** |  51 s   | **clean rc.d architecture**  |

The direct-launch path is slightly faster but architecturally worse
(bypasses rc.d).  Use the rc.d/igen_attach path for production.
