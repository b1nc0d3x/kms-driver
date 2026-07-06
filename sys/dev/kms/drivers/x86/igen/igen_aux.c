/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Haswell DisplayPort AUX channel transport and EDID-over-I2C-over-AUX.
 *
 * Per-DDI AUX register block lives at 0x64010 + ddi*0x100 (DDI A at
 * 0x64010, DDI B at 0x64110, …, DDI E at 0x64410).  Layout, all 32-bit:
 *   CTL   (+0x00)  send/done/error/precharge/size/clock-divider
 *   DATA0 (+0x04)  packed header + first 4 data bytes
 *   DATA1 (+0x08)  next 4 data bytes
 *   DATA2 (+0x0c)
 *   DATA3 (+0x10)
 *   DATA4 (+0x14)
 *
 * AUX_CTL bits (HSW BSpec vol4 DDI_AUX_CTL):
 *   31    SEND_BUSY    1 = transfer in flight; W1S to launch; HW W1C
 *   30    DONE         W1C; set when transfer completes (ack or error)
 *   29    INTERRUPT_ON_DONE
 *   28    TIME_OUT_ERROR  W1C
 *   27:26 TIME_OUT_TIMER  00=400us, 01=600us, 10=800us, 11=1600us
 *   25    RECEIVE_ERROR W1C
 *   24    RESERVED
 *   23:20 MSG_SIZE      bytes in the request / received in reply
 *   19:16 PRECHARGE_TIME  count of 2us periods (HSW: typically 5)
 *   15:0  BIT_CLOCK_2X_DIVIDER  divisor that produces ~2 MHz AUX bit clock
 *
 * BIT_CLOCK_2X_DIVIDER is computed as DIV_ROUND_CLOSEST(cdclk_khz, 2000)
 * on HSW per i915.  CDCLK comes from LCPLL_CTL[27:26]: at 540 MHz the
 * divider is 270 (0x10E); at 450 MHz it's 225 (0xE1); at 337.5 it's 169
 * (0xA9).  We re-read LCPLL each transfer so a userspace CDCLK change
 * (BDW+ supports it) doesn't desync the AUX clock.
 *
 * DPCD/I2C-over-AUX request packet layout (DP 1.4 §2.7.5.1):
 *   byte 0:    (cmd << 4) | (addr >> 16)         — for I2C, addr <= 0xff
 *                                                  so top nibble is 0
 *   byte 1-2:  addr[15:8], addr[7:0]
 *   byte 3:    (size - 1)                        — read or write count
 *   byte 4..:  write payload (write commands only)
 *
 * Reply: first byte is the AUX status nibble in [3:0] (native and I2C
 * use different reply encodings — DP_AUX_NATIVE_REPLY_* in low 2 bits,
 * DP_AUX_I2C_REPLY_* in bits[3:2]).  The remaining bytes are read data.
 *
 * For EDID we use I2C-over-AUX with MOT (Middle-Of-Transaction) bits to
 * keep the bus open between the address-setting write and the data
 * read.  Apple eDP panels typically NAK any attempt that omits MOT.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>

#include <machine/bus.h>

#include <kms/drm_dp_helper.h>

#include "igen_internal.h"
#include "igen_aux.h"

/* HSW AUX register offsets, indexed by DDI 0..4 (A..E). */
#define	HSW_AUX_CTL(d)		(0x00064010u + (d) * 0x100u)
#define	HSW_AUX_DATA(d, i)	(0x00064014u + (d) * 0x100u + (i) * 4u)

#define	HSW_AUX_CTL_SEND_BUSY		(1u << 31)
#define	HSW_AUX_CTL_DONE		(1u << 30)
#define	HSW_AUX_CTL_INTERRUPT_ON_DONE	(1u << 29)
#define	HSW_AUX_CTL_TIME_OUT_ERROR	(1u << 28)
#define	HSW_AUX_CTL_TIME_OUT_1600US	(3u << 26)
#define	HSW_AUX_CTL_RECEIVE_ERROR	(1u << 25)
#define	HSW_AUX_CTL_MSG_SIZE_SHIFT	20
#define	HSW_AUX_CTL_MSG_SIZE_MASK	(0x1fu << 20)
#define	HSW_AUX_CTL_PRECHARGE_SHIFT	16
#define	HSW_AUX_CTL_PRECHARGE_5US	(5u << 16)
#define	HSW_AUX_CTL_BIT_CLOCK_2X_MASK	0xffffu

/* LCPLL register (also defined in igen_dpll.c — kept private here too). */
#define	HSW_LCPLL_CTL_LOCAL		0x00130040u

/*
 * Pack the 4-byte AUX header into the high bytes of DATA0.  HSW's
 * DATA registers are big-endian in their on-wire interpretation: byte 0
 * (the command/upper-addr nibble) maps to bits[31:24], byte 1 to [23:16],
 * byte 2 to [15:8], byte 3 to [7:0].
 */
static inline uint32_t
igen_aux_pack_header(uint8_t request, uint32_t address, uint8_t size_minus_1)
{
	return (((uint32_t)((request << 4) | ((address >> 16) & 0xf)) << 24) |
	        ((uint32_t)((address >> 8) & 0xff) << 16) |
	        ((uint32_t)(address & 0xff) << 8) |
	        (uint32_t)size_minus_1);
}

/*
 * BIT_CLOCK_2X divider from current CDCLK.  Read LCPLL_CTL[27:26]:
 *   00 = 450, 01 = 540, 10 = 337.5, 11 = 675 MHz
 * Divider produces 2 MHz AUX bit clock: cdclk_kHz / 2000.
 */
static uint16_t
igen_hsw_aux_clock_divider(struct igen_softc *sc)
{
	uint32_t lcpll = igen_r32(sc, HSW_LCPLL_CTL_LOCAL);
	uint32_t cdclk_khz;

	switch ((lcpll >> 26) & 3) {
	case 0:	cdclk_khz = 450000; break;
	case 1:	cdclk_khz = 540000; break;
	case 2:	cdclk_khz = 337500; break;
	case 3:	cdclk_khz = 675000; break;
	default: cdclk_khz = 450000; break;
	}
	/* DIV_ROUND_CLOSEST(cdclk_khz, 2000). */
	return ((uint16_t)((cdclk_khz + 1000) / 2000));
}

/*
 * Single AUX transfer.  Send up to 16 data bytes, receive up to 20
 * (1 status + 19 data) per DP 1.4 §2.7.5.5.  msg->reply is set to the
 * raw status byte; msg->buffer is updated with read data on success.
 * Returns bytes of data transferred (excluding status), or negative
 * errno on hard error.  Defer / NACK retry is the caller's job
 * (kms_dp_aux_transfer for native, igen_aux_i2c_xfer for I2C-MOT).
 */
ssize_t
igen_aux_transfer(struct drm_dp_aux *aux, struct drm_dp_aux_msg *msg)
{
	struct igen_aux_channel *ch = aux->priv;
	struct igen_softc *sc = ch->sc;
	uint32_t header;
	uint32_t ctl;
	uint32_t data[5];
	bool is_write = (msg->request == DP_AUX_NATIVE_WRITE) ||
	    (msg->request == DP_AUX_I2C_WRITE) ||
	    (msg->request == (DP_AUX_I2C_WRITE | DP_AUX_I2C_MOT)) ||
	    (msg->request == (DP_AUX_NATIVE_WRITE | DP_AUX_I2C_MOT));
	size_t tx_bytes;	/* full packet on the wire */
	size_t i;
	int j;

	if (msg->size > 16)
		return (-E2BIG);

	tx_bytes = 4 + (is_write ? msg->size : 0);

	memset(data, 0, sizeof(data));
	header = igen_aux_pack_header(msg->request, msg->address,
	    msg->size == 0 ? 0 : (uint8_t)(msg->size - 1));
	data[0] = header;
	if (is_write && msg->size > 0) {
		uint8_t *buf = msg->buffer;
		for (i = 0; i < msg->size; i++) {
			size_t byte_idx = 4 + i;	/* offset into packet */
			size_t reg_idx = byte_idx / 4;
			size_t reg_shift = (3 - (byte_idx % 4)) * 8;
			data[reg_idx] |= ((uint32_t)buf[i]) << reg_shift;
		}
	}
	for (j = 0; j < 5; j++)
		igen_w32(sc, HSW_AUX_DATA(ch->ddi, j), data[j]);

	/* W1C all status bits before launching.  Read-modify-write. */
	ctl = igen_r32(sc, HSW_AUX_CTL(ch->ddi));
	ctl |= HSW_AUX_CTL_DONE | HSW_AUX_CTL_TIME_OUT_ERROR |
	    HSW_AUX_CTL_RECEIVE_ERROR;
	igen_w32(sc, HSW_AUX_CTL(ch->ddi), ctl);

	/* Build the launch CTL value and W1S SEND_BUSY. */
	ctl = HSW_AUX_CTL_SEND_BUSY |
	    HSW_AUX_CTL_TIME_OUT_1600US |
	    HSW_AUX_CTL_PRECHARGE_5US |
	    ((uint32_t)tx_bytes << HSW_AUX_CTL_MSG_SIZE_SHIFT) |
	    (uint32_t)igen_hsw_aux_clock_divider(sc);
	igen_w32(sc, HSW_AUX_CTL(ch->ddi), ctl);

	/* Poll for completion.  HSW with TIME_OUT=1600us means worst-case
	 * a single bit-clock cycle plus the precharge — 10 ms is generous. */
	for (i = 0; i < 1000; i++) {
		ctl = igen_r32(sc, HSW_AUX_CTL(ch->ddi));
		if ((ctl & HSW_AUX_CTL_SEND_BUSY) == 0)
			break;
		DELAY(10);
	}
	if (ctl & HSW_AUX_CTL_SEND_BUSY) {
		DPRINTF(sc, 1, "aux ddi%d: timeout waiting SEND_BUSY clear\n",
		    ch->ddi);
		return (-ETIMEDOUT);
	}
	if (ctl & (HSW_AUX_CTL_TIME_OUT_ERROR | HSW_AUX_CTL_RECEIVE_ERROR)) {
		DPRINTF(sc, 2,
		    "aux ddi%d: ctl=0x%08x time_out=%d recv_err=%d\n",
		    ch->ddi, ctl,
		    (ctl & HSW_AUX_CTL_TIME_OUT_ERROR) ? 1 : 0,
		    (ctl & HSW_AUX_CTL_RECEIVE_ERROR) ? 1 : 0);
		/* W1C the error bits so the next transfer starts clean. */
		igen_w32(sc, HSW_AUX_CTL(ch->ddi), ctl);
		return (-EIO);
	}

	/* MSG_SIZE in the reply tells us the byte count delivered. */
	size_t reply_bytes = (ctl & HSW_AUX_CTL_MSG_SIZE_MASK) >>
	    HSW_AUX_CTL_MSG_SIZE_SHIFT;
	if (reply_bytes == 0) {
		msg->reply = 0;
		return (0);
	}

	/* Read back DATA[0..n] and unpack. */
	for (j = 0; j < 5 && j * 4 < (int)reply_bytes; j++)
		data[j] = igen_r32(sc, HSW_AUX_DATA(ch->ddi, j));

	/* Reply byte 0 = top byte of DATA0 = AUX status. */
	msg->reply = (uint8_t)(data[0] >> 24);

	/* Remaining reply bytes are payload, byte 1 onward. */
	size_t payload = reply_bytes - 1;
	if (payload > msg->size)
		payload = msg->size;
	if (payload > 0 && msg->buffer != NULL && !is_write) {
		uint8_t *buf = msg->buffer;
		for (i = 0; i < payload; i++) {
			size_t byte_idx = 1 + i;
			size_t reg_idx = byte_idx / 4;
			size_t reg_shift = (3 - (byte_idx % 4)) * 8;
			buf[i] = (uint8_t)(data[reg_idx] >> reg_shift);
		}
	}

	/* W1C the DONE bit so the controller is idle for next call. */
	igen_w32(sc, HSW_AUX_CTL(ch->ddi),
	    ctl | HSW_AUX_CTL_DONE);

	return ((ssize_t)payload);
}

/*
 * I2C-over-AUX transfer with DP-specific defer/NACK handling.  Unlike
 * the kms_dp_aux_transfer helper (native-AUX retry semantics), the
 * I2C reply bits live in [3:2].  Up to 7 defer retries per DP 1.4
 * §2.7.5.5; partial replies on a multi-byte read are normal — sinks
 * are allowed to deliver fewer bytes than requested and the caller
 * loops.
 */
static int
igen_aux_i2c_xfer(struct drm_dp_aux *aux, uint8_t request, uint32_t addr,
    void *buffer, size_t size)
{
	struct drm_dp_aux_msg msg;
	ssize_t ret;
	int retries;

	memset(&msg, 0, sizeof(msg));
	msg.request = request;
	msg.address = addr;
	msg.buffer = buffer;
	msg.size = size;

	for (retries = 0; retries < 7; retries++) {
		ret = igen_aux_transfer(aux, &msg);
		if (ret < 0)
			return ((int)ret);
		switch (msg.reply & 0x0c) {	/* I2C reply bits [3:2] */
		case DP_AUX_I2C_REPLY_ACK:
			return ((int)ret);
		case DP_AUX_I2C_REPLY_NACK:
			return (-EIO);
		case DP_AUX_I2C_REPLY_DEFER:
			DELAY(500);
			continue;
		}
		/*
		 * Some sinks misuse the native reply bits even for I2C
		 * transactions when the address phase NAKs.  Treat the
		 * native NACK position as a hard error to avoid spinning.
		 */
		if ((msg.reply & 0x03) == DP_AUX_NATIVE_REPLY_NACK)
			return (-EIO);
	}
	return (-ETIMEDOUT);
}

/*
 * Read `size` bytes from I2C device `i2c_addr` (7-bit) at offset 0.
 * Uses the MOT bit to keep the bus open across the address-set write
 * and the payload reads, finishing with a non-MOT terminator so the
 * sink releases the bus and resets internal state.  Caller-friendly
 * wrapper for EDID-over-DDC — the typical path on DP/eDP.
 */
int
igen_aux_i2c_read_block(struct drm_dp_aux *aux, uint8_t i2c_addr,
    uint8_t offset, uint8_t *buf, size_t size)
{
	int error;
	size_t done = 0;

	/*
	 * Address-set write (1 byte: the EDID start offset).  MOT keeps
	 * the I2C bus open between this write and the subsequent reads,
	 * which DP sinks treat as a single combined transaction.
	 */
	error = igen_aux_i2c_xfer(aux,
	    DP_AUX_I2C_WRITE | DP_AUX_I2C_MOT, i2c_addr,
	    &offset, 1);
	if (error < 0)
		return (error);

	/* Read in 16-byte chunks (AUX max payload per transfer). */
	while (done < size) {
		size_t chunk = size - done;
		if (chunk > 16)
			chunk = 16;
		error = igen_aux_i2c_xfer(aux,
		    DP_AUX_I2C_READ | DP_AUX_I2C_MOT, i2c_addr,
		    buf + done, chunk);
		if (error < 0)
			return (error);
		if (error == 0) {
			/* Sink delivered zero — stop short, real EDID will
			 * have its checksum off but we've made forward
			 * progress observable to the caller. */
			break;
		}
		done += (size_t)error;
	}

	/* Final stop transaction (no MOT) — frees the I2C bus. */
	(void)igen_aux_i2c_xfer(aux, DP_AUX_I2C_WRITE, i2c_addr, NULL, 0);

	return ((int)done);
}

/*
 * Initialise an AUX channel struct for the given DDI (0=A..4=E).  Wires
 * up the transfer callback + priv so the kms framework can drive it
 * via kms_dp_aux_transfer / kms_dp_dpcd_read.
 */
void
igen_aux_init(struct igen_softc *sc, struct igen_aux_channel *ch, int ddi,
    const char *name)
{
	ch->sc = sc;
	ch->ddi = ddi;
	ch->aux.name = name;
	ch->aux.transfer = igen_aux_transfer;
	ch->aux.priv = ch;
	kms_dp_aux_init(&ch->aux);
}
