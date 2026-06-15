/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * igen PCH GMBus driver + EDID read sysctl.  Split out of igen.c
 * for size + topical cohesion: the I2C transfer layer for all DDI DDC
 * paths, plus the user-triggered sysctl that exercises it.  The
 * EDID -> drm_display_mode parser stays in igen.c since it's tightly
 * coupled to the connector attach flow.
 *
 * Exported entry points:
 *   int  igen_gmbus_read_block(sc, pin, slave, offset, buf, len);
 *   void igen_gmbus_register_sysctls(sc);
 *
 * Sysctl registered here:
 *   dev.igen.<n>.re.edid_read_b   =1 reads 128 EDID bytes on DDI_B,
 *                                  =2 sweeps pins 1..9, =3 wakes
 *                                  DDI_BUF_B then retries pin 5/4,
 *                                  =4 sweeps slave addresses on pin 4.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>

#include "igen_internal.h"

/* ---------------------------- GMBus / EDID -------------------------------- */

/*
 * SKL/KBL PCH GMBus.  Lives in display south block at PCH_DISPLAY_BASE +
 * 0x5100 = 0xc5100.  All DDI DDC paths funnel through this one controller;
 * which physical DDI is reached depends on the GMBUS0 port-number field
 * (pin pair).  i915's pin-pair numbering for SKL+ north display:
 *   1 = SSC   (panel sense)
 *   2 = VGADDC
 *   3 = PANEL
 *   4 = DDI_C   (HDMI-C / DP-C)
 *   5 = DDI_B   (HDMI-B / DP-B / SDVO)
 *   6 = DDI_D   (HDMI-D / DP-D)
 */

#define	GMBUS0			0x000c5100
#define	GMBUS1			0x000c5104
#define	GMBUS2			0x000c5108
#define	GMBUS3			0x000c510c
#define	GMBUS4			0x000c5110
#define	GMBUS5			0x000c5120

/*
 * SKL/KBL/CFL Display WA #0868: PCH GMBus unit clock is gated by default
 * on Sunrise Point / Kaby Point PCH.  Without explicitly disabling the
 * gate before a transaction, GMBUS0 writes go through but the IOs never
 * drive — pins look electrically dead with no NAK and no HW_RDY.
 * Bit 31 of SOUTH_DSPCLK_GATE_D is the gate-disable.
 */
#define	SOUTH_DSPCLK_GATE_D		0x000c2020
#define	  PCH_GMBUSUNIT_CLK_GATE_DIS	(1u << 31)

#define	GMBUS_RATE_100KHZ	(0 << 8)
#define	GMBUS_BYTE_CNT_OVERRIDE	(1u << 6)	/* GMBUS0 — allow >9 byte */
#define	GMBUS_PIN_DDI_C		4
#define	GMBUS_PIN_DDI_B		5
#define	GMBUS_PIN_DDI_D		6

#define	GMBUS_SW_CLR_INT	(1u << 31)
#define	GMBUS_SW_RDY		(1u << 30)
#define	GMBUS_CYCLE_WAIT	(1u << 25)	/* wait for next phase */
#define	GMBUS_CYCLE_STOP	(4u << 25)	/* stop on completion */
#define	GMBUS_BYTE_COUNT_SHIFT	16
#define	GMBUS_SLAVE_ADDR_SHIFT	1
#define	GMBUS_SLAVE_READ	1
#define	GMBUS_SLAVE_WRITE	0

#define	GMBUS_HW_RDY		(1u << 11)
#define	GMBUS_HW_WAIT		(1u << 14)
#define	GMBUS_NAK		(1u << 10)
#define	GMBUS_ACTIVE		(1u << 9)
#define	GMBUS_INUSE		(1u << 15)

#define	EDID_SLAVE		0x50

#define	DDI_BUF_CTL_A		0x64000
#define	DDI_BUF_CTL_B		0x64100
#define	DDI_BUF_CTL_C		0x64200
#define	DDI_BUF_CTL_D		0x64300
#define	DDI_BUF_CTL_E		0x64400
#define	DDI_BUF_CTL_ENABLE	(1u << 31)
#define	DDI_BUF_IS_IDLE		(1u << 7)

static void
igen_ddi_buf_wake(struct igen_softc *sc, uint32_t buf_ctl_reg)
{
	uint32_t v = igen_r32(sc, buf_ctl_reg);

	device_printf(sc->dev, "ddi_buf 0x%05x: pre=0x%08x (idle=%d)\n",
	    buf_ctl_reg, v, (v & DDI_BUF_IS_IDLE) != 0);

	/*
	 * Touch ENABLE again; i915 sometimes does this explicitly to wake
	 * the DDC line.  Then poll for !IDLE.  IDLE clear ~500us after
	 * ENABLE rises per BSpec.
	 */
	igen_w32(sc, buf_ctl_reg, v | DDI_BUF_CTL_ENABLE);
	for (int i = 0; i < 100; i++) {
		v = igen_r32(sc, buf_ctl_reg);
		if ((v & DDI_BUF_IS_IDLE) == 0)
			break;
		DELAY(10);
	}
	device_printf(sc->dev, "ddi_buf 0x%05x: post=0x%08x (idle=%d)\n",
	    buf_ctl_reg, v, (v & DDI_BUF_IS_IDLE) != 0);
}

static int
igen_gmbus_wait(struct igen_softc *sc, uint32_t bit)
{
	uint32_t s;

	for (int spin = 0; spin < 50000; spin++) {
		s = igen_r32(sc, GMBUS2);
		if (s & GMBUS_NAK)
			return (EIO);
		if (s & bit)
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}

int
igen_gmbus_read_block(struct igen_softc *sc, uint32_t pin,
    uint8_t slave, uint8_t offset, uint8_t *buf, size_t len)
{
	uint32_t cmd, val;
	int error;
	size_t got = 0;

	if (len == 0 || len > 256)
		return (EINVAL);

	/*
	 * Display WA #0868: disable PCH GMBus clock gating.  Keep it
	 * disabled across transactions — toggling per-xfer leaves the
	 * controller in a transient state that causes the next xfer to
	 * NAK or wedge.  Power cost is negligible for a polled DDC path.
	 */
	uint32_t gate = igen_r32(sc, SOUTH_DSPCLK_GATE_D);
	if ((gate & PCH_GMBUSUNIT_CLK_GATE_DIS) == 0)
		igen_w32(sc, SOUTH_DSPCLK_GATE_D,
		    gate | PCH_GMBUSUNIT_CLK_GATE_DIS);

	/*
	 * Quiesce the controller: clear any stuck interrupt, ensure 2-byte
	 * index off, then park GMBUS0=0 before reprogramming the pin.
	 * GMBUS2 INUSE is write-1-to-clear; without explicitly W1C-ing it
	 * a failed prior transaction (or BIOS hand-off) wedges every
	 * subsequent xfer at HW_RDY because the bus stays "in use".
	 */
	igen_w32(sc, GMBUS0, 0);
	igen_w32(sc, GMBUS4, 0);
	igen_w32(sc, GMBUS5, 0);
	igen_w32(sc, GMBUS1, GMBUS_SW_CLR_INT);
	igen_w32(sc, GMBUS1, 0);
	if (igen_r32(sc, GMBUS2) & GMBUS_INUSE) {
		DPRINTF(sc, 1, "gmbus: INUSE stuck, clearing\n");
		igen_w32(sc, GMBUS2, GMBUS_INUSE);
	}
	igen_w32(sc, GMBUS0, pin | GMBUS_RATE_100KHZ);

	/*
	 * Phase 1: write the EDID byte offset (segment 0).  CYCLE_WAIT
	 * keeps the bus owned past this transaction so the read phase
	 * issues a repeated START rather than a fresh STOP/START.
	 */
	cmd = GMBUS_SW_RDY | GMBUS_CYCLE_WAIT |
	    ((uint32_t)1 << GMBUS_BYTE_COUNT_SHIFT) |
	    ((uint32_t)slave << GMBUS_SLAVE_ADDR_SHIFT) |
	    GMBUS_SLAVE_WRITE;
	igen_w32(sc, GMBUS3, offset);
	igen_w32(sc, GMBUS1, cmd);
	error = igen_gmbus_wait(sc, GMBUS_HW_WAIT);
	if (error != 0) {
		DPRINTF(sc, 1, "gmbus: wait HW_WAIT after addr: %d\n", error);
		goto out;
	}
	/*
	 * EDID EEPROMs need a settle delay between offset-pointer write and
	 * the read phase; without it the EEPROM clocks out stale-pointer
	 * data (often 0x00) for the first few bytes.
	 */
	DELAY(500);

	/*
	 * Phase 2: read `len` bytes.  CYCLE_WAIT (not STOP) here matches
	 * i915: it keeps the bus open for the whole multi-byte read and
	 * relies on the unconditional STOP terminator in the `out:` block
	 * to drive STOP on the wire after the last byte.
	 */
	cmd = GMBUS_SW_RDY | GMBUS_CYCLE_WAIT |
	    ((uint32_t)len << GMBUS_BYTE_COUNT_SHIFT) |
	    ((uint32_t)slave << GMBUS_SLAVE_ADDR_SHIFT) |
	    GMBUS_SLAVE_READ;
	igen_w32(sc, GMBUS1, cmd);

	while (got < len) {
		error = igen_gmbus_wait(sc, GMBUS_HW_RDY);
		if (error != 0) {
			uint32_t s = igen_r32(sc, GMBUS2);
			DPRINTF(sc, 1,
			    "gmbus: wait HW_RDY at %zu/%zu: %d  GMBUS2=0x%08x\n",
			    got, len, error, s);
			goto out;
		}
		val = igen_r32(sc, GMBUS3);
		for (int i = 0; i < 4 && got < len; i++)
			buf[got++] = (val >> (i * 8)) & 0xff;
	}
out:
	/*
	 * Per i915: always terminate with an explicit CYCLE_STOP+SW_RDY
	 * write to GMBUS1 to drive STOP on the wire and re-park the FSM,
	 * even on success.  Then tri-state pin, W1C INUSE, leave the
	 * clock-gate disabled for the next xfer.
	 */
	igen_w32(sc, GMBUS1, GMBUS_SW_RDY | GMBUS_CYCLE_STOP);
	(void)igen_gmbus_wait(sc, GMBUS_HW_WAIT);
	igen_w32(sc, GMBUS0, 0);
	igen_w32(sc, GMBUS1, GMBUS_SW_CLR_INT);
	igen_w32(sc, GMBUS1, 0);
	igen_w32(sc, GMBUS2, GMBUS_INUSE);
	return (error);
}

static int
igen_sysctl_edid_read_b(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	uint8_t edid[128];
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (trigger == 0)
		return (0);

	/*
	 * trigger=4: scan slave addresses on the only electrically alive
	 * pin (pin 4) with a 1-byte read.  ACK without NAK = slave present.
	 */
	if (trigger == 4) {
		for (uint8_t s = 0x08; s <= 0x77; s++) {
			uint8_t one = 0;
			uint32_t cmd, val, snap;
			error = 0;

			igen_w32(sc, GMBUS0, 0);
			igen_w32(sc, GMBUS4, 0);
			igen_w32(sc, GMBUS5, 0);
			igen_w32(sc, GMBUS1, GMBUS_SW_CLR_INT);
			igen_w32(sc, GMBUS1, 0);
			if (igen_r32(sc, GMBUS2) & GMBUS_INUSE)
				igen_w32(sc, GMBUS2, GMBUS_INUSE);
			igen_w32(sc, GMBUS0, 4 | GMBUS_RATE_100KHZ);

			cmd = GMBUS_SW_RDY | GMBUS_CYCLE_STOP |
			    ((uint32_t)1 << GMBUS_BYTE_COUNT_SHIFT) |
			    ((uint32_t)s << GMBUS_SLAVE_ADDR_SHIFT) |
			    GMBUS_SLAVE_READ;
			igen_w32(sc, GMBUS1, cmd);

			for (int spin = 0; spin < 5000; spin++) {
				snap = igen_r32(sc, GMBUS2);
				if (snap & (GMBUS_NAK | GMBUS_HW_RDY |
				    GMBUS_HW_WAIT))
					break;
				DELAY(10);
			}
			val = (snap & GMBUS_HW_RDY) ?
			    igen_r32(sc, GMBUS3) : 0;
			one = val & 0xff;
			if ((snap & GMBUS_NAK) == 0) {
				device_printf(sc->dev,
				    "scan: slave 0x%02x ACK!  GMBUS2=0x%08x"
				    "  byte0=0x%02x\n", s, snap, one);
			}
			igen_w32(sc, GMBUS0, 0);
		}
		device_printf(sc->dev, "scan done\n");
		return (0);
	}

	/*
	 * trigger=3: wake DDI_BUF_CTL_B then read EDID on pin 5 (canonical
	 * SKL+ DDI_B pin).  Goal: prove that the live HDMI port's DDC line
	 * is gated on DDI_BUF being active rather than just having ENABLE
	 * set passively.  Fallback to pin 4 if pin 5 still times out.
	 */
	if (trigger == 3) {
		igen_ddi_buf_wake(sc, DDI_BUF_CTL_B);
		DELAY(2000);
		for (uint32_t pin = 5; pin >= 4; pin--) {
			memset(edid, 0, sizeof(edid));
			error = igen_gmbus_read_block(sc, pin,
			    EDID_SLAVE, 0, edid, 16);
			device_printf(sc->dev,
			    "post-wake pin %u: err=%d  first16:"
			    " %02x %02x %02x %02x %02x %02x %02x %02x"
			    "  %02x %02x %02x %02x %02x %02x %02x %02x\n",
			    pin, error,
			    edid[0], edid[1], edid[2], edid[3],
			    edid[4], edid[5], edid[6], edid[7],
			    edid[8], edid[9], edid[10], edid[11],
			    edid[12], edid[13], edid[14], edid[15]);
			if (error == 0)
				break;
		}
		return (0);
	}

	/*
	 * Pin sweep: try every GMBUS pin in [1..9] with the EDID slave.
	 * The first pin that completes without NAK is the one wired to the
	 * physical DDC line of the active connector.  This bypasses any
	 * static assumption about port->pin mapping.
	 */
	if (trigger == 2) {
		for (uint32_t pin = 1; pin <= 9; pin++) {
			memset(edid, 0, sizeof(edid));
			error = igen_gmbus_read_block(sc, pin, EDID_SLAVE,
			    0, edid, 16);
			device_printf(sc->dev,
			    "pin %u: err=%d  first16: %02x %02x %02x %02x"
			    " %02x %02x %02x %02x  %02x %02x %02x %02x"
			    " %02x %02x %02x %02x\n", pin, error,
			    edid[0], edid[1], edid[2], edid[3],
			    edid[4], edid[5], edid[6], edid[7],
			    edid[8], edid[9], edid[10], edid[11],
			    edid[12], edid[13], edid[14], edid[15]);
		}
		return (0);
	}

	memset(edid, 0, sizeof(edid));
	error = igen_gmbus_read_block(sc, GMBUS_PIN_DDI_B, EDID_SLAVE,
	    0, edid, sizeof(edid));
	if (error != 0) {
		device_printf(sc->dev,
		    "edid_read_b: gmbus failed: %d  partial: "
		    "%02x %02x %02x %02x %02x %02x %02x %02x\n", error,
		    edid[0], edid[1], edid[2], edid[3],
		    edid[4], edid[5], edid[6], edid[7]);
		return (0);
	}
	device_printf(sc->dev, "edid_read_b: 128 bytes read:\n");
	for (int row = 0; row < 16; row++) {
		device_printf(sc->dev,
		    "  %02x: %02x %02x %02x %02x %02x %02x %02x %02x\n",
		    row * 8,
		    edid[row*8+0], edid[row*8+1], edid[row*8+2], edid[row*8+3],
		    edid[row*8+4], edid[row*8+5], edid[row*8+6], edid[row*8+7]);
	}
	/* Quick sanity decode: bytes 8-9 = manufacturer, 54+ = first DTD. */
	device_printf(sc->dev,
	    "  header OK=%d  mfg=%02x%02x  prod=%02x%02x  serial=%02x%02x%02x%02x\n",
	    (edid[0] == 0x00 && edid[1] == 0xff && edid[2] == 0xff &&
	     edid[7] == 0x00),
	    edid[8], edid[9], edid[10], edid[11],
	    edid[12], edid[13], edid[14], edid[15]);
	return (0);
}

/*
 * Register the GMBus-driven sysctls under dev.igen.<n>.re.  Called
 * from igen_re_sysctls_init in igen.c.
 */
void
igen_gmbus_register_sysctls(struct igen_softc *sc)
{
	struct sysctl_ctx_list *ctx = &sc->re_sysctl_ctx;
	struct sysctl_oid_list *children =
	    SYSCTL_CHILDREN(sc->re_sysctl_tree);

	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "edid_read_b",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_edid_read_b, "I",
	    "write 1 to GMBus-read 128 bytes of EDID block 0 from DDI_B");
}
