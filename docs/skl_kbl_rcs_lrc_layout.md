# SKL/KBL gen 9 RCS Logical Ring Context layout

Reference extract from Linux i915 v4.19 `intel_lrc.c` (`populate_lr_context`,
`execlists_init_reg_state`).  Used to build `igen_gt_compose_lrc`.

## High-level

- LRC total size ≈ `engine->context_size` (~0xD000 / 52 KB for RCS) +
  `LRC_HEADER_PAGES * PAGE_SIZE` (0x2000 / 8 KB) = ~60 KB.
- Page 0 (offset 0x0000): reserved engine header — DMA scratch.
- Page 1 (offset 0x1000): **register state image** — the LRI sequence
  documented below.  This is what the engine reads to restore state.
- Pages 2+: rest of context (URB, etc.).

## Register state image at LRC + 0x1000

Three back-to-back MI_LOAD_REGISTER_IMM blocks.  Each writes
engine-local offsets and the value the engine should load on
restore.  All offsets are relative to `RING_BASE_RCS = 0x2000`.

### LRI Header 0 — 14 register pairs

```
[0x00]  CTX_LRI_HEADER_0    = 0x11000014   (MI_LRI(14) | FORCE_POSTED)
[0x04]  CONTEXT_CONTROL     = 0x244
[0x08]  ctx_control_value   = 0x401800C8
        /* _MASKED_BIT_DISABLE(0x18) | _MASKED_BIT_ENABLE(0x401)
         * = clear (CTX_RESTORE_INHIBIT | CTX_SAVE_INHIBIT)
         *   set   (INHIBIT_SYN_CTX_SWITCH | RS_CTX_ENABLE) */
[0x0C]  RING_HEAD           = 0x34
[0x10]  ring_head_value     = 0 (set at submit time)
[0x14]  RING_TAIL           = 0x30
[0x18]  ring_tail_value     = batch byte length (set at submit time)
[0x1C]  RING_START          = 0x38
[0x20]  ring_start_value    = GGTT address of ring buffer
[0x24]  RING_CTL            = 0x3C
[0x28]  ring_ctl_value      = ((pages - 1) << 12) | RING_VALID(0x1)
[0x2C]  RING_BBADDR_UDW     = 0x168
[0x30]  bb_head_u_value     = 0
[0x34]  RING_BBADDR         = 0x140
[0x38]  bb_head_l_value     = 0
[0x3C]  RING_BBSTATE        = 0x110
[0x40]  bb_state_value      = RING_BB_PPGTT (0x20)
[0x44]  RING_SBBADDR_UDW    = 0x16C
[0x48]  second_bb_head_u    = 0
[0x4C]  RING_SBBADDR        = 0x144
[0x50]  second_bb_head_l    = 0
[0x54]  RING_SBBSTATE       = 0x114
[0x58]  second_bb_state     = 0
[0x5C]  RING_INDIRECT_CTX           = 0x1BC  /* RCS only */
[0x60]  indirect_ctx_value          = 0 (or workaround batch ptr + size)
[0x64]  RING_INDIRECT_CTX_OFFSET    = 0x1C8  /* RCS only */
[0x68]  indirect_ctx_offset_value   = 0
[0x6C]  RING_BB_PER_CTX_PTR         = 0x1C0  /* RCS only */
[0x70]  bb_per_ctx_value            = 0
```

### LRI Header 1 — 9 register pairs (CTX_TIMESTAMP + 4 PPGTT PDPs)

```
[0x74]  CTX_LRI_HEADER_1            = 0x11000009  (MI_LRI(9) | FORCE_POSTED)
[0x78]  RING_CTX_TIMESTAMP          = 0x3A8
[0x7C]  ctx_timestamp_value         = 0
[0x80]  GEN8_RING_PDP_UDW(3)        = 0x270 + 3*8 + 4 = 0x28C
[0x84]  pdp3_udw_value              = 0
[0x88]  GEN8_RING_PDP_LDW(3)        = 0x270 + 3*8     = 0x288
[0x8C]  pdp3_ldw_value              = 0
[0x90]  GEN8_RING_PDP_UDW(2)        = 0x284
[0x94]  pdp2_udw_value              = 0
[0x98]  GEN8_RING_PDP_LDW(2)        = 0x280
[0x9C]  pdp2_ldw_value              = 0
[0xA0]  GEN8_RING_PDP_UDW(1)        = 0x27C
[0xA4]  pdp1_udw_value              = 0
[0xA8]  GEN8_RING_PDP_LDW(1)        = 0x278
[0xAC]  pdp1_ldw_value              = 0
[0xB0]  GEN8_RING_PDP_UDW(0)        = 0x274
[0xB4]  pdp0_udw_value              = 0
[0xB8]  GEN8_RING_PDP_LDW(0)        = 0x270
[0xBC]  pdp0_ldw_value              = 0
```

For our no-op batch we leave PDPs zero — the batch is GGTT-resident
so PPGTT translation is bypassed.

### LRI Header 2 — 1 register pair (R_PWR_CLK_STATE / RPCS)

```
[0xC0]  CTX_LRI_HEADER_2            = 0x11000001  (MI_LRI(1) | FORCE_POSTED)
[0xC4]  GEN8_R_PWR_CLK_STATE        = 0x0C8
[0xC8]  rpcs_value                  = make_rpcs(dev_priv)
        /* slice/subslice/EU mask; can be 0 to leave HW defaults */
```

After RPCS comes an MI_BATCH_BUFFER_END or zeros — engine stops
processing LRI data when it sees a non-LRI opcode.

## Context descriptor (gen 9)

64-bit field written as two 32-bit halves to ELSP:

```
bit 63..32: LRCA (LRC GGTT address, page-aligned, low 32 bits — i.e.,
                  the GGTT byte offset of the LRC, which fits in 32 bits
                  for our setup since GGTT is ≤ 4 GiB)
bit 31..16: reserved
bit 15..12: engine_class (RCS = 0, BCS = 1, VCS = 2, VECS = 3)
bit 11:     PRIVILEGE
bit 10:     IRQ_DISABLE
bit 9..8:   addressing_mode (0 = ADDRESSING_MODE_LEGACY_64B,
                              1 = ADDRESSING_MODE_LEGACY_32B,
                              2 = ADDRESSING_MODE_LEGACY_48B - SKL+ default)
bit 1:      FAULT_DISABLE
bit 0:      VALID
```

i915 uses ADDRESSING_MODE_LEGACY_48B (2 << 8 = 0x200) for SKL+ with
the 48-bit PML4 PPGTT.  For our PPGTT-bypass first batch we can leave
LEGACY_64B (0x0) and zero PDPs.

## ELSP write order (gen 9)

Four 32-bit writes to ELSP per submit, in this order:

```
write 1: ctx1_descriptor_hi   (or 0 if no second context)
write 2: ctx1_descriptor_lo
write 3: ctx0_descriptor_hi
write 4: ctx0_descriptor_lo
```

After the 4th write, the engine pops both descriptors off the
2-deep submission queue.  Poll `EXECLIST_STATUS_RCS` bit 7 (or
0x2234 bit 0..2 = "submission queue contents") to confirm.

## Forcewake (correct addresses)

```
FORCEWAKE_RENDER_GEN9       = 0xA188
FORCEWAKE_RENDER_GEN9_ACK   = 0x0D84
FORCEWAKE_MEDIA_GEN9        = 0xA270
FORCEWAKE_MEDIA_GEN9_ACK    = 0x0D88
FORCEWAKE_BLITTER_GEN9      = 0xA188  (shared register with render)
```

Request pattern: write `((1<<bit) << 16) | (1<<bit)` to the request
register (the high 16 bits are a SET mask).  Poll bit 0 of the ACK
register to come high.  Release pattern: write `((1<<bit) << 16)`
(zero in the low 16 means CLEAR for that bit).

## Source

Linux kernel v4.19 `drivers/gpu/drm/i915/intel_lrc.c`, functions
`populate_lr_context` and `execlists_init_reg_state`.  Read for
reference, BSD-license-clean (not copied verbatim).
