/*
 * Written by Oron Peled <oron@actcom.co.il>
 * Copyright (C) 2004-2006, Xorcom
 *
 * Parts derived from Cologne demo driver for the chip.
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include "xpd.h"
#include "xproto.h"
#include "xpp_zap.h"
#include "card_bri.h"
#include "zap_debug.h"
#include "xpd.h"
#include "xbus-core.h"

static const char rcsid[] = "$Id: card_bri.c 3957 2008-03-07 00:45:53Z tzafrir $";

#ifndef CONFIG_ZAPATA_BRI_DCHANS
#error CONFIG_ZAPATA_BRI_DCHANS is not defined
#endif

DEF_PARM(int, print_dbg, 0, 0644, "Print DBG statements");	/* must be before zap_debug.h */
DEF_PARM(uint, poll_interval, 500, 0644, "Poll channel state interval in milliseconds (0 - disable)");
DEF_PARM_BOOL(nt_keepalive, 1, 0644, "Force BRI_NT to keep trying connection");

enum xhfc_states {
	ST_RESET		= 0,	/* G/F0	*/
	/* TE */
	ST_TE_SENSING		= 2,	/* F2	*/
	ST_TE_DEACTIVATED	= 3,	/* F3	*/
	ST_TE_SIGWAIT		= 4,	/* F4	*/
	ST_TE_IDENT		= 5,	/* F5	*/
	ST_TE_SYNCED		= 6,	/* F6	*/
	ST_TE_ACTIVATED		= 7,	/* F7	*/
	ST_TE_LOST_FRAMING	= 8,	/* F8	*/
	/* NT */
	ST_NT_DEACTIVATED	= 1,	/* G1	*/
	ST_NT_ACTIVATING	= 2,	/* G2	*/
	ST_NT_ACTIVATED		= 3,	/* G3	*/
	ST_NT_DEACTIVTING	= 4,	/* G4	*/
};

static const char *xhfc_state_name(xpd_type_t xpd_type, enum xhfc_states state)
{
	const char	*p;

#define	_E(x)	[ST_ ## x] = #x
	static const char *te_names[] = {
		_E(RESET),
		_E(TE_SENSING),
		_E(TE_DEACTIVATED),
		_E(TE_SIGWAIT),
		_E(TE_IDENT),
		_E(TE_SYNCED),
		_E(TE_ACTIVATED),
		_E(TE_LOST_FRAMING),
	};
	static const char *nt_names[] = {
		_E(RESET),
		_E(NT_DEACTIVATED),
		_E(NT_ACTIVATING),
		_E(NT_ACTIVATED),
		_E(NT_DEACTIVTING),
	};
#undef	_E
	if(xpd_type == XPD_TYPE_BRI_TE) {
		if ((state < ST_RESET) || (state > ST_TE_LOST_FRAMING))
			p = "TE ???";
		else
			p = te_names[state];
	} else {
		if ((state < ST_RESET) || (state > ST_NT_DEACTIVTING))
			p = "NT ???";
		else
			p = nt_names[state];
	}
	return p;
}

/* xhfc Layer1 physical commands */
#define HFC_L1_ACTIVATE_TE		0x01
#define HFC_L1_FORCE_DEACTIVATE_TE	0x02
#define HFC_L1_ACTIVATE_NT		0x03
#define HFC_L1_DEACTIVATE_NT		0x04

#define HFC_L1_ACTIVATING	1
#define HFC_L1_ACTIVATED	2
#define	TIMER_T1_MAX		2500
#define	HFC_TIMER_T3		8000	/* 8s activation timer T3 */
#define	HFC_TIMER_T4		500	/* 500ms deactivation timer T4 */
#define	HFC_TIMER_OFF		-1	/* timer disabled */

#define	A_SU_WR_STA		0x30	/* ST/Up state machine register		*/
#define		V_SU_LD_STA	0x10
#define 	V_SU_ACT	0x60	/* start activation/deactivation	*/
#define 	STA_DEACTIVATE	0x40	/* start deactivation in A_SU_WR_STA */
#define 	STA_ACTIVATE	0x60	/* start activation   in A_SU_WR_STA */
#define 	V_SU_SET_G2_G3	0x80

#define	A_SU_RD_STA		0x30
typedef union {
	struct {
		byte	v_su_sta:4;
		byte	v_su_fr_sync:1;
		byte	v_su_t2_exp:1;
		byte	v_su_info0:1;
		byte	v_g2_g3:1;
	} bits;
	byte	reg;
} su_rd_sta_t;

#define	REG30_LOST	3	/* in polls */
#define	DCHAN_LOST	15000	/* in ticks */

#define	BRI_DCHAN_SIGCAP	(			  \
					ZT_SIG_EM	| \
					ZT_SIG_CLEAR	| \
					ZT_SIG_FXSLS	| \
					ZT_SIG_FXSGS	| \
					ZT_SIG_FXSKS	| \
					ZT_SIG_FXOLS	| \
					ZT_SIG_FXOGS	| \
					ZT_SIG_FXOKS	| \
					ZT_SIG_CAS	| \
					ZT_SIG_SF	  \
				)
#define	BRI_BCHAN_SIGCAP	ZT_SIG_CLEAR

#define	IS_NT(xpd)		((xpd)->type == XPD_TYPE_BRI_NT)

/* shift in PCM highway */
#define	SUBUNIT_PCM_SHIFT	4
#define	PCM_SHIFT(mask, sunit)	((mask) << (SUBUNIT_PCM_SHIFT * (sunit)))

/*---------------- BRI Protocol Commands ----------------------------------*/

static int write_state_register(xpd_t *xpd, byte value);
static bool bri_packet_is_valid(xpacket_t *pack);
static void bri_packet_dump(const char *msg, xpacket_t *pack);
static int proc_bri_info_read(char *page, char **start, off_t off, int count, int *eof, void *data);
static int proc_xpd_register_read(char *page, char **start, off_t off, int count, int *eof, void *data);
static int proc_xpd_register_write(struct file *file, const char __user *buffer, unsigned long count, void *data);
static int bri_spanconfig(struct zt_span *span, struct zt_lineconfig *lc);
static int bri_chanconfig(struct zt_chan *chan, int sigtype);
static int bri_startup(struct zt_span *span);
static int bri_shutdown(struct zt_span *span);

#define	PROC_REGISTER_FNAME	"slics"
#define	PROC_BRI_INFO_FNAME	"bri_info"

#define	VALID_CHIPSEL(x)	((x) == 0)

enum led_state {
	BRI_LED_OFF		= 0x0,
	BRI_LED_ON		= 0x1,
	/*
	 * We blink by software from driver, so that
	 * if the driver malfunction that blink would stop.
	 */
	// BRI_LED_BLINK_SLOW	= 0x2,	/* 1/2 a second blink cycle */
	// BRI_LED_BLINK_FAST	= 0x3	/* 1/4 a second blink cycle */
};

enum bri_led_names {
	GREEN_LED	= 0,
	RED_LED		= 1
};

#define	NUM_LEDS	2
#define	LED_TICKS	100


struct bri_leds {
	byte	state:2;
	byte	led_sel:1;	/* 0 - GREEN, 1 - RED */
	byte	reserved:5;
};

#ifndef MAX_DFRAME_LEN_L1
#define MAX_DFRAME_LEN_L1 300
#endif

#define	DCHAN_BUFSIZE	MAX_DFRAME_LEN_L1

struct BRI_priv_data {
	struct proc_dir_entry		*regfile;
	struct proc_dir_entry		*bri_info;
	su_rd_sta_t			state_register;
	bool				initialized;
	int				t1; /* timer 1 for NT deactivation */
	int				t3; /* timer 3 for activation */
	int				t4; /* timer 4 for deactivation */
	ulong				l1_flags;
	bool				reg30_good;
	uint				reg30_ticks;
	bool				layer1_up;
	xpp_line_t			card_pcm_mask;

	/*
	 * D-Chan: buffers + extra state info.
	 */
	int				dchan_r_idx;
	byte				dchan_rbuf[DCHAN_BUFSIZE];
	byte				dchan_tbuf[DCHAN_BUFSIZE];
	bool				txframe_begin;

	reg_cmd_t			requested_reply;
	reg_cmd_t			last_reply;
	uint				tick_counter;
	uint				poll_counter;
	uint				dchan_tx_counter;
	uint				dchan_rx_counter;
	uint				dchan_rx_drops;
	bool				dchan_alive;
	uint				dchan_alive_ticks;
	uint				dchan_notx_ticks;
	uint				dchan_norx_ticks;
	enum led_state			ledstate[NUM_LEDS];
};

static xproto_table_t	PROTO_TABLE(BRI_NT);
static xproto_table_t	PROTO_TABLE(BRI_TE);


DEF_RPACKET_DATA(BRI, SET_LED,	/* Set one of the LED's */
	struct bri_leds		bri_leds;
	);

static /* 0x33 */ DECLARE_CMD(BRI, SET_LED, enum bri_led_names which_led, enum led_state to_led_state);

#define	DO_LED(xpd, which, tostate)	\
			CALL_PROTO(BRI, SET_LED, (xpd)->xbus, (xpd), (which), (tostate))

#define DEBUG_BUF_SIZE (100)
static void dump_hex_buf(xpd_t *xpd, char *msg, byte *buf, size_t len)
{
	char	debug_buf[DEBUG_BUF_SIZE + 1];
	int	i;
	int	n = 0;

	debug_buf[0] = '\0';
	for(i = 0; i < len && n < DEBUG_BUF_SIZE; i++)
		n += snprintf(&debug_buf[n], DEBUG_BUF_SIZE - n, "%02X ", buf[i]);
	XPD_DBG(GENERAL, xpd, "%s[0..%zd]: %s%s\n", msg, len-1, debug_buf,
			(n >= DEBUG_BUF_SIZE)?"...":"");
}

static void dump_dchan_packet(xpd_t *xpd, bool transmit, byte *buf, int len)
{
	struct BRI_priv_data	*priv;
	char	msgbuf[MAX_PROC_WRITE];
	char	ftype = '?';
	char	*direction;
	int	frame_begin;

	priv = xpd->priv;
	BUG_ON(!priv);
	if(transmit) {
		direction = "TX";
		frame_begin = priv->txframe_begin;
	} else {
		direction = "RX";
		frame_begin = 1;
	}
	if(frame_begin) {	/* Packet start */
		if(!IS_SET(buf[0], 7))
			ftype = 'I';	/* Information */
		else if(IS_SET(buf[0], 7) && !IS_SET(buf[0], 6))
			ftype = 'S';	/* Supervisory */
		else if(IS_SET(buf[0], 7) && IS_SET(buf[0], 6))
			ftype = 'U';	/* Unnumbered */
		else
			XPD_NOTICE(xpd, "Unknown frame type 0x%X\n", buf[0]);

		snprintf(msgbuf, MAX_PROC_WRITE, "D-Chan %s = (%c) ", direction, ftype);
	} else {
		snprintf(msgbuf, MAX_PROC_WRITE, "D-Chan %s =     ", direction);
	}
	dump_hex_buf(xpd, msgbuf, buf, len);
}

static void layer1_state(xpd_t *xpd, bool up)
{
	struct BRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	if(priv->layer1_up == up)
		return;
	priv->layer1_up = up;
	XPD_DBG(SIGNAL, xpd, "STATE CHANGE: Layer1 %s\n", (up)?"UP":"DOWN");
}

static void dchan_state(xpd_t *xpd, bool up)
{
	struct BRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	if(priv->dchan_alive == up)
		return;
	if(up) {
		XPD_DBG(SIGNAL, xpd, "STATE CHANGE: D-Channel RUNNING\n");
		priv->dchan_alive = 1;
	} else {
		XPD_DBG(SIGNAL, xpd, "STATE CHANGE: D-Channel STOPPED\n");
		priv->dchan_rx_counter = priv->dchan_tx_counter = priv->dchan_rx_drops = 0;
		priv->dchan_alive = 0;
		priv->dchan_alive_ticks = 0;
	}
}

static void xpd_activation(xpd_t *xpd, bool on)
{
	struct BRI_priv_data	*priv;
	xbus_t			*xbus;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	xbus = xpd->xbus;
	XPD_DBG(SIGNAL, xpd, "%s\n", (on)?"ON":"OFF");
	switch(xpd->type) {
		case XPD_TYPE_BRI_TE:
			if(on) {
				XPD_DBG(SIGNAL, xpd, "HFC_L1_ACTIVATE_TE\n");
				set_bit(HFC_L1_ACTIVATING, &priv->l1_flags);
				write_state_register(xpd, STA_ACTIVATE);
				priv->t3 = HFC_TIMER_T3;
			} else {
				XPD_DBG(SIGNAL, xpd, "HFC_L1_FORCE_DEACTIVATE_TE\n");
				write_state_register(xpd, STA_DEACTIVATE);
			}
			break;
		case XPD_TYPE_BRI_NT:
			if(on) {
				XPD_DBG(SIGNAL, xpd, "HFC_L1_ACTIVATE_NT\n");
				priv->t1 = TIMER_T1_MAX;
				set_bit(HFC_L1_ACTIVATING, &priv->l1_flags);
				write_state_register(xpd, STA_ACTIVATE | V_SU_SET_G2_G3);
			} else {
				XPD_DBG(SIGNAL, xpd, "HFC_L1_DEACTIVATE_NT\n");
				write_state_register(xpd, STA_DEACTIVATE);
			}
			break;
		default:
			XPD_ERR(xpd, "%s: Bad xpd type %d\n", __FUNCTION__, xpd->type);
			BUG();
	}
}


/*
 * D-Chan receive
 */
static int rx_dchan(xpd_t *xpd, reg_cmd_t *regcmd)
{
	xbus_t			*xbus;
	struct BRI_priv_data	*priv;
	byte			*src;
	byte			*dst;
	byte			*dchan_buf;
	struct zt_chan		*dchan;
	uint			len;
	bool			eoframe;
	int			idx;
	int			ret = 0;

	src = REG_XDATA(regcmd);
	len = regcmd->bytes;
	eoframe = regcmd->eoframe;
	if(len <= 0)
		return 0;
	if(!SPAN_REGISTERED(xpd)) /* Nowhere to copy data */
		return 0;
	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	xbus = xpd->xbus;
#ifdef XPP_DEBUGFS
	xbus_log(xbus, xpd, 0, regcmd, sizeof(reg_cmd_t));		/* 0 = RX */
#endif
	dchan = &xpd->span.chans[2];
	if(!IS_SET(xpd->offhook, 2)) {	/* D-chan is used? */
		static int rate_limit;

		if((rate_limit++ % 1000) == 0)
			XPD_DBG(SIGNAL, xpd, "D-Chan unused\n");
		dchan->bytes2receive = 0;
		dchan->bytes2transmit = 0;
		goto out;
	}
	dchan_buf = dchan->readchunk;
	idx = priv->dchan_r_idx;
	if(idx + len >= DCHAN_BUFSIZE) {
		XPD_ERR(xpd, "D-Chan RX overflow: %d\n", idx);
		dump_hex_buf(xpd, "    current packet", src, len);
		dump_hex_buf(xpd, "    dchan_buf", dchan_buf, idx);
		ret = -ENOSPC;
		if(eoframe)
			goto drop;
		goto out;
	}
	dst = dchan_buf + idx;
	idx += len;
	priv->dchan_r_idx = idx;
	memcpy(dst, src, len);
	if(!eoframe)
		goto out;
	if(idx < 4) {
		XPD_NOTICE(xpd, "D-Chan RX short frame (idx=%d)\n", idx);
		dump_hex_buf(xpd, "D-Chan RX:    current packet", src, len);
		dump_hex_buf(xpd, "D-Chan RX:    chan_buf", dchan_buf, idx);
		ret = -EPROTO;
		goto drop;
	}
	if(dchan_buf[idx-1]) {
		XPD_NOTICE(xpd, "D-Chan RX Bad checksum: [%02X:%02X=%02X] (%d)\n",
				dchan_buf[idx-3], dchan_buf[idx-2], dchan_buf[idx-1], dchan_buf[idx-1]);
		dump_hex_buf(xpd, "D-Chan RX:    current packet", src, len);
		dump_hex_buf(xpd, "D-Chan RX:    chan_buf", dchan_buf, idx);
		ret = -EPROTO;
		goto drop;
	}
	if(print_dbg)
		dump_dchan_packet(xpd, 0, dchan_buf, idx /* - 3 */);	/* Print checksum? */
	/* 
	 * Tell Zaptel that we received idx-1 bytes. They include the data and a 2-byte checksum.
	 * The last byte (that we don't pass on) is 0 if the checksum is correct. If it were wrong,
	 * we would drop the packet in the "if(dchan_buf[idx-1])" above.
	 */
	dchan->bytes2receive = idx - 1;
	dchan->eofrx = 1;
	priv->dchan_rx_counter++;
	priv->dchan_norx_ticks = 0;
drop:
	priv->dchan_r_idx = 0;
out:
	return ret;
}

static int send_bri_multibyte(xpd_t *xpd, byte *buf, int len, bool eoftx)
{
	xbus_t		*xbus = xpd->xbus;
	xframe_t	*xframe;
	xpacket_t	*pack;
	reg_cmd_t	*reg_cmd;
	int		ret;

	BUG_ON(len < 0);
	/*
	 * Zero length multibyte is legal and has special meaning for the
	 * firmware:
	 *   eoftx==1: Start sending us D-channel packets.
	 *   eoftx==0: Stop sending us D-channel packets.
	 */
	if(len > MULTIBYTE_MAX_LEN) {
		XPD_ERR(xpd, "%s: len=%d is too long. dropping.\n", __FUNCTION__, len);
		return -EINVAL;
	}
	XFRAME_NEW_CMD(xframe, pack, xbus, GLOBAL, REGISTER_REQUEST, xpd->xbus_idx);
	reg_cmd = &RPACKET_FIELD(pack, GLOBAL, REGISTER_REQUEST, reg_cmd);
	reg_cmd->bytes = len;
	reg_cmd->eoframe = eoftx;
	reg_cmd->multibyte = 1;
	if(len > 0) {
		memcpy(REG_XDATA(reg_cmd), (byte *)buf, len);
	} else {
		XPD_DBG(REGS, xpd, "Magic Packet (eoftx=%d)\n", eoftx);
	}
#ifdef XPP_DEBUGFS
	xbus_log(xbus, xpd, 1, reg_cmd, sizeof(reg_cmd_t));	/* 1 = TX */
#else
	if(print_dbg)
		dump_xframe("SEND_BRI_MULTI", xbus, xframe, print_dbg);
#endif
	ret = send_cmd_frame(xbus, xframe);
	if(ret < 0)
		XPD_NOTICE(xpd, "%s: failed sending xframe\n", __FUNCTION__);
	return ret;
}

/*
 * D-Chan transmit
 */
static int tx_dchan(xpd_t *xpd)
{
	struct BRI_priv_data	*priv;
	struct zt_chan		*dchan;
	int			len;
	int			eoframe;
	int			ret;

	priv = xpd->priv;
	BUG_ON(!priv);
	if(!IS_NT(xpd)) {
		static int	rate_limit;

		if (priv->t3 > HFC_TIMER_OFF) {
			/* timer expired ? */
			if (--priv->t3 == 0) {
				if ((rate_limit % 1003) >= 5)
					XPD_DBG(SIGNAL, xpd, "T3 expired\n");
				priv->t3 = HFC_TIMER_OFF;
				clear_bit(HFC_L1_ACTIVATING, &priv->l1_flags);
				xpd_activation(xpd, 0);		/* Deactivate TE */
			}
		}
		if (priv->t4 > HFC_TIMER_OFF) {
			/* timer expired ? */
			if (--priv->t4 == 0) {
				if ((rate_limit % 1003) >= 5)
					XPD_DBG(SIGNAL, xpd, "T4 expired\n");
				priv->t4 = HFC_TIMER_OFF;
			}
		}
		rate_limit++;
	}
	if(!SPAN_REGISTERED(xpd) || !(xpd->span.flags & ZT_FLAG_RUNNING))
		return 0;
	dchan = &xpd->chans[2];
	len = dchan->bytes2transmit;	/* dchan's hdlc package len */
	eoframe = dchan->eoftx;		/* dchan's end of frame */
	dchan->bytes2transmit = 0;
	dchan->eoftx = 0;
	dchan->bytes2receive = 0;
	dchan->eofrx = 0;
	if(len <= 0)
		return 0; /* Nothing to transmit on D channel */
	if(len > MULTIBYTE_MAX_LEN) {
		XPD_ERR(xpd, "%s: len=%d. need to split. Unimplemented.\n", __FUNCTION__, len);
		return -EINVAL;
	}
	if(!test_bit(HFC_L1_ACTIVATED, &priv->l1_flags) && !test_bit(HFC_L1_ACTIVATING, &priv->l1_flags)) {
		XPD_DBG(SIGNAL, xpd, "Want to transmit: Kick D-Channel transmiter\n");
		xpd_activation(xpd, 1);
		return 0;
	}
	if(print_dbg)
		dump_dchan_packet(xpd, 1, priv->dchan_tbuf, len);
	if(eoframe)
		priv->txframe_begin = 1;
	else
		priv->txframe_begin = 0;
	ret = send_bri_multibyte(xpd, priv->dchan_tbuf, len, eoframe);
	if(ret < 0)
		XPD_NOTICE(xpd, "%s: failed sending xframe\n", __FUNCTION__);
	if(eoframe)
		priv->dchan_tx_counter++;
	priv->dchan_notx_ticks = 0;
	return ret;
}

/*---------------- BRI: Methods -------------------------------------------*/

static xpd_t *BRI_card_new(xbus_t *xbus, int unit, int subunit, const xproto_table_t *proto_table, byte subtype, byte revision)
{
	xpd_t		*xpd = NULL;
	int		channels = min(3, CHANNELS_PERXPD);

	XBUS_DBG(GENERAL, xbus, "\n");
	xpd = xpd_alloc(sizeof(struct BRI_priv_data), proto_table, channels);
	if(!xpd)
		return NULL;
	xpd->direction = (proto_table == &PROTO_TABLE(BRI_NT)) ? TO_PHONE : TO_PSTN;
	xpd->revision = revision;
	xpd->type_name = proto_table->name;
	return xpd;
}

static void clean_proc(xbus_t *xbus, xpd_t *xpd)
{
	struct BRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	XPD_DBG(PROC, xpd, "\n");
#ifdef	CONFIG_PROC_FS
	if(priv->regfile) {
		XPD_DBG(PROC, xpd, "Removing registers file\n");
		priv->regfile->data = NULL;
		remove_proc_entry(PROC_REGISTER_FNAME, xpd->proc_xpd_dir);
	}
	if(priv->bri_info) {
		XPD_DBG(PROC, xpd, "Removing xpd BRI_INFO file\n");
		remove_proc_entry(PROC_BRI_INFO_FNAME, xpd->proc_xpd_dir);
	}
#endif
}

static int BRI_card_init(xbus_t *xbus, xpd_t *xpd)
{
	struct BRI_priv_data	*priv;
	int			ret = 0;

	BUG_ON(!xpd);
	XPD_DBG(GENERAL, xpd, "\n");
	priv = xpd->priv;
#ifdef	CONFIG_PROC_FS
	XPD_DBG(PROC, xpd, "Creating BRI_INFO file\n");
	priv->bri_info = create_proc_read_entry(PROC_BRI_INFO_FNAME, 0444, xpd->proc_xpd_dir, proc_bri_info_read, xpd);
	if(!priv->bri_info) {
		XPD_ERR(xpd, "Failed to create proc file '%s'\n", PROC_BRI_INFO_FNAME);
		ret = -ENOENT;
		goto err;
	}
	priv->bri_info->owner = THIS_MODULE;
	XPD_DBG(PROC, xpd, "Creating registers file\n");
	priv->regfile = create_proc_entry(PROC_REGISTER_FNAME, 0644, xpd->proc_xpd_dir);
	if(!priv->regfile) {
		XPD_ERR(xpd, "Failed to create proc file '%s'\n", PROC_REGISTER_FNAME);
		goto err;
	}
	priv->regfile->owner = THIS_MODULE;
	priv->regfile->write_proc = proc_xpd_register_write;
	priv->regfile->read_proc = proc_xpd_register_read;
	priv->regfile->data = xpd;
#endif
	priv->t1 = HFC_TIMER_OFF;
	ret = run_initialize_registers(xpd);
	if(ret < 0)
		goto err;
	XPD_DBG(PROC, xpd, "done\n");
	priv->initialized = 1;
	return 0;
err:
	clean_proc(xbus, xpd);
	XPD_ERR(xpd, "Failed initializing registers (%d)\n", ret);
	return ret;
}

static int BRI_card_remove(xbus_t *xbus, xpd_t *xpd)
{
	struct BRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	XPD_DBG(GENERAL, xpd, "\n");
	clean_proc(xbus, xpd);
	return 0;
}

static int BRI_card_zaptel_preregistration(xpd_t *xpd, bool on)
{
	xbus_t			*xbus;
	struct BRI_priv_data	*priv;
	xpp_line_t		tmp_pcm_mask;
	int			tmp_pcm_len;
	unsigned long		flags;
	int			i;
	
	BUG_ON(!xpd);
	xbus = xpd->xbus;
	priv = xpd->priv;
	BUG_ON(!xbus);
	XPD_DBG(GENERAL, xpd, "%s\n", (on)?"on":"off");
	if(!on) {
		/* Nothing to do yet */
		return 0;
	}
#ifdef ZT_SPANSTAT_V2 
	xpd->span.spantype = "BRI";
#endif 
	xpd->span.linecompat = ZT_CONFIG_AMI | ZT_CONFIG_CCS;
	xpd->span.deflaw = ZT_LAW_ALAW;
	BIT_SET(xpd->digital_signalling, 2);	/* D-Channel */
	for_each_line(xpd, i) {
		struct zt_chan	*cur_chan = &xpd->chans[i];

		XPD_DBG(GENERAL, xpd, "setting BRI channel %d\n", i);
		snprintf(cur_chan->name, MAX_CHANNAME, "XPP_%s/%02d/%1d%1d/%d",
				xpd->xproto->name, xbus->num,
				xpd->addr.unit, xpd->addr.subunit, i);
		cur_chan->chanpos = i + 1;
		cur_chan->pvt = xpd;
		if(i == 2) {	/* D-CHAN */
			cur_chan->sigcap = BRI_DCHAN_SIGCAP;
			cur_chan->flags |= ZT_FLAG_BRIDCHAN;
			cur_chan->flags &= ~ZT_FLAG_HDLC;

			/* Setup big buffers for D-Channel rx/tx */
			cur_chan->readchunk = priv->dchan_rbuf;
			cur_chan->writechunk = priv->dchan_tbuf;
			priv->dchan_r_idx = 0;
			priv->txframe_begin = 1;

			cur_chan->maxbytes2transmit = MULTIBYTE_MAX_LEN;
			cur_chan->bytes2transmit = 0;
			cur_chan->bytes2receive = 0;
		} else
			cur_chan->sigcap = BRI_BCHAN_SIGCAP;
	}
	xpd->offhook = BIT(0) | BIT(1);	/* 2*bchan */

	/*
	 * Compute PCM lentgh and mask
	 * We know all cards have been initialized until now
	 */
	tmp_pcm_mask = 0;
	if(xpd->addr.subunit == 0) {
		int	line_count = 0;

		for(i = 0; i < MAX_SUBUNIT; i++) {
			xpd_t	*sub_xpd = xpd_byaddr(xbus, xpd->addr.unit, i);
			if(sub_xpd) {
				tmp_pcm_mask |= PCM_SHIFT(sub_xpd->wanted_pcm_mask, i);
				line_count += 2;
			}
		}
		tmp_pcm_len = RPACKET_HEADERSIZE + sizeof(xpp_line_t)  +  line_count * ZT_CHUNKSIZE;
	} else
		tmp_pcm_len = 0;
	spin_lock_irqsave(&xpd->lock, flags);
	xpd->pcm_len = tmp_pcm_len;
	xpd->wanted_pcm_mask = xpd->offhook;
	priv->card_pcm_mask = tmp_pcm_mask;
	xpd->span.spanconfig = bri_spanconfig;
	xpd->span.chanconfig = bri_chanconfig;
	xpd->span.startup = bri_startup;
	xpd->span.shutdown = bri_shutdown;
	spin_unlock_irqrestore(&xpd->lock, flags);
	return 0;
}

static int BRI_card_zaptel_postregistration(xpd_t *xpd, bool on)
{
	xbus_t			*xbus;
	struct BRI_priv_data	*priv;
	
	BUG_ON(!xpd);
	xbus = xpd->xbus;
	priv = xpd->priv;
	BUG_ON(!xbus);
	XPD_DBG(GENERAL, xpd, "%s\n", (on)?"on":"off");
	return(0);
}

static int BRI_card_hooksig(xbus_t *xbus, xpd_t *xpd, int pos, zt_txsig_t txsig)
{
	LINE_DBG(SIGNAL, xpd, pos, "%s\n", txsig2str(txsig));
	return 0;
}

/*
 * LED managment is done by the driver now:
 *   - Turn constant ON RED/GREEN led to indicate NT/TE port
 *   - Very fast "Double Blink" to indicate Layer1 alive (without D-Channel)
 *   - Constant blink (1/2 sec cycle) to indicate D-Channel alive.
 */
static void handle_leds(xbus_t *xbus, xpd_t *xpd)
{
	struct BRI_priv_data	*priv;
	unsigned int		timer_count;
	int			which_led;
	int			other_led;
	int			mod;

	BUG_ON(!xpd);
	if(IS_NT(xpd)) {
		which_led = RED_LED;
		other_led = GREEN_LED;
	} else {
		which_led = GREEN_LED;
		other_led = RED_LED;
	}
	priv = xpd->priv;
	BUG_ON(!priv);
	timer_count = xpd->timer_count;
	if(xpd->blink_mode) {
		if((timer_count % DEFAULT_LED_PERIOD) == 0) {
			// led state is toggled
			if(priv->ledstate[which_led] == BRI_LED_OFF) {
				DO_LED(xpd, which_led, BRI_LED_ON);
				DO_LED(xpd, other_led, BRI_LED_ON);
			} else {
				DO_LED(xpd, which_led, BRI_LED_OFF);
				DO_LED(xpd, other_led, BRI_LED_OFF);
			}
		}
		return;
	}
	if(priv->ledstate[other_led] != BRI_LED_OFF)
		DO_LED(xpd, other_led, BRI_LED_OFF);
	if(priv->dchan_alive) {
		mod = timer_count % 1000;
		switch(mod) {
			case 0:
				DO_LED(xpd, which_led, BRI_LED_ON);
				break;
			case 500:
				DO_LED(xpd, which_led, BRI_LED_OFF);
				break;
		}
	} else if(priv->layer1_up) {
		mod = timer_count % 1000;
		switch(mod) {
			case 0:
			case 100:
				DO_LED(xpd, which_led, BRI_LED_ON);
				break;
			case 50:
			case 150:
				DO_LED(xpd, which_led, BRI_LED_OFF);
				break;
		}
	} else {
		if(priv->ledstate[which_led] != BRI_LED_ON)
			DO_LED(xpd, which_led, BRI_LED_ON);
	}
}

/* Poll the register ST/Up-State-machine Register, to see if the cable
 * if a cable is connected to the port.
 */
static int BRI_card_tick(xbus_t *xbus, xpd_t *xpd)
{
	struct BRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	if(!priv->initialized || !xbus->self_ticking)
		return 0;
	if(poll_interval != 0 && (priv->tick_counter % poll_interval) == 0) {
		// XPD_DBG(GENERAL, xpd, "%d\n", priv->tick_counter);
		priv->poll_counter++;
		xpp_register_request(xbus, xpd, 0, 0, 0, A_SU_RD_STA, 0, 0, 0);

		if(IS_NT(xpd) && nt_keepalive &&
			!test_bit(HFC_L1_ACTIVATED, &priv->l1_flags) &&
			!test_bit(HFC_L1_ACTIVATING, &priv->l1_flags)) {
			XPD_DBG(SIGNAL, xpd, "Kick NT D-Channel\n");
			xpd_activation(xpd, 1);
		}
	}
	/* Detect D-Channel disconnect heuristic */
	priv->dchan_notx_ticks++;
	priv->dchan_norx_ticks++;
	priv->dchan_alive_ticks++;
	if(priv->dchan_alive && (priv->dchan_notx_ticks > DCHAN_LOST || priv->dchan_norx_ticks > DCHAN_LOST)) {
		/*
		 * No tx_dchan() or rx_dchan() for many ticks
		 * This D-Channel is probabelly dead.
		 */
		dchan_state(xpd, 0);
	} else if(priv->dchan_rx_counter > 1 &&  priv->dchan_tx_counter > 1) {
		if(!priv->dchan_alive)
			dchan_state(xpd, 1);
	}
	/* Detect Layer1 disconnect */
	if(priv->reg30_good && priv->reg30_ticks > poll_interval * REG30_LOST) {
		/* No reply for 1/2 a second */
		XPD_ERR(xpd, "Lost state tracking for %d ticks\n", priv->reg30_ticks);
		priv->reg30_good = 0;
		layer1_state(xpd, 0);
		dchan_state(xpd, 0);
	}
	handle_leds(xbus, xpd);
	tx_dchan(xpd);
	/* Detect T1 timer expiry on NT */
	if(IS_NT(xpd) && !nt_keepalive) {
		if (priv->t1 > HFC_TIMER_OFF) {
			if (--priv->t1 == 0) {
				XPD_DBG(SIGNAL, xpd, "T1 Expired. Deactivate NT\n");
				priv->t1 = HFC_TIMER_OFF;
				clear_bit(HFC_L1_ACTIVATING, &priv->l1_flags);
				write_state_register(xpd, STA_DEACTIVATE);
			}
		}
	}
	priv->tick_counter++;
	priv->reg30_ticks++;
	return 0;
}

static int BRI_card_ioctl(xpd_t *xpd, int pos, unsigned int cmd, unsigned long arg)
{
	BUG_ON(!xpd);
	if(!TRANSPORT_RUNNING(xpd->xbus))
		return -ENODEV;
	switch (cmd) {
		case ZT_TONEDETECT:
			/*
			 * Asterisk call all span types with this (FXS specific)
			 * call. Silently ignore it.
			 */
			LINE_DBG(SIGNAL, xpd, pos, "BRI: Starting a call\n");
			return -ENOTTY;
		default:
			report_bad_ioctl(THIS_MODULE->name, xpd, pos, cmd);
			return -ENOTTY;
	}
	return 0;
}

static int BRI_card_close(xpd_t *xpd, lineno_t pos)
{
	struct zt_chan	*chan = &xpd->span.chans[pos];

	/* Clear D-Channel pending data */
	chan->bytes2receive = 0;
	chan->eofrx = 0;
	chan->bytes2transmit = 0;
	chan->eoftx = 0;
	return 0;
}

/*
 * Called only for 'span' keyword in /etc/zaptel.conf
 */
static int bri_spanconfig(struct zt_span *span, struct zt_lineconfig *lc)
{
	xpd_t		*xpd = span->pvt;
	const char	*framingstr = "";
	const char	*codingstr = "";
	const char	*crcstr = "";

	/* framing first */
	if (lc->lineconfig & ZT_CONFIG_B8ZS)
		framingstr = "B8ZS";
	else if (lc->lineconfig & ZT_CONFIG_AMI)
		framingstr = "AMI";
	else if (lc->lineconfig & ZT_CONFIG_HDB3)
		framingstr = "HDB3";
	/* then coding */
	if (lc->lineconfig & ZT_CONFIG_ESF)
		codingstr = "ESF";
	else if (lc->lineconfig & ZT_CONFIG_D4)
		codingstr = "D4";
	else if (lc->lineconfig & ZT_CONFIG_CCS)
		codingstr = "CCS";
	/* E1's can enable CRC checking */
	if (lc->lineconfig & ZT_CONFIG_CRC4)
		crcstr = "CRC4";
	XPD_DBG(GENERAL, xpd, "[%s]: span=%d (%s) lbo=%d lineconfig=%s/%s/%s (0x%X) sync=%d\n",
		IS_NT(xpd)?"NT":"TE",
		lc->span,
		lc->name,
		lc->lbo,
		framingstr, codingstr, crcstr,
		lc->lineconfig,
		lc->sync);
	/*
	 * FIXME: validate
	 */
	span->lineconfig = lc->lineconfig;
	return 0;
}

/*
 * Set signalling type (if appropriate)
 * Called from zaptel with spinlock held on chan. Must not call back
 * zaptel functions.
 */
static int bri_chanconfig(struct zt_chan *chan, int sigtype)
{
	DBG(GENERAL, "channel %d (%s) -> %s\n", chan->channo, chan->name, sig2str(sigtype));
	// FIXME: sanity checks:
	// - should be supported (within the sigcap)
	// - should not replace fxs <->fxo ??? (covered by previous?)
	return 0;
}

/*
 * Called only for 'span' keyword in /etc/zaptel.conf
 */
static int bri_startup(struct zt_span *span)
{
	xpd_t			*xpd = span->pvt;
	struct BRI_priv_data	*priv;
	struct zt_chan		*dchan;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	if(!TRANSPORT_RUNNING(xpd->xbus)) {
		XPD_DBG(GENERAL, xpd, "Startup called by zaptel. No Hardware. Ignored\n");
		return -ENODEV;
	}
	XPD_DBG(GENERAL, xpd, "STARTUP\n");
	// Turn on all channels
	CALL_XMETHOD(XPD_STATE, xpd->xbus, xpd, 1);
	write_state_register(xpd, 0);	/* Enable L1 state machine */
	xpd_activation(xpd, 1);
	if(SPAN_REGISTERED(xpd)) {
		dchan = &span->chans[2];
		span->flags |= ZT_FLAG_RUNNING;
		/*
		 * Zaptel (wrongly) assume that D-Channel need HDLC decoding
		 * and during zaptel registration override our flags.
		 *
		 * Don't Get Mad, Get Even:  Now we override zaptel :-)
		 */
		dchan->flags |= ZT_FLAG_BRIDCHAN;
		dchan->flags &= ~ZT_FLAG_HDLC;
	}
	return 0;
}

/*
 * Called only for 'span' keyword in /etc/zaptel.conf
 */
static int bri_shutdown(struct zt_span *span)
{
	xpd_t			*xpd = span->pvt;
	struct BRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	if(!TRANSPORT_RUNNING(xpd->xbus)) {
		XPD_DBG(GENERAL, xpd, "Shutdown called by zaptel. No Hardware. Ignored\n");
		return -ENODEV;
	}
	XPD_DBG(GENERAL, xpd, "SHUTDOWN\n");
	// Turn off all channels
	CALL_XMETHOD(XPD_STATE, xpd->xbus, xpd, 0);
	if(IS_NT(xpd))
		xpd_activation(xpd, 0);
	return 0;
}

static void BRI_card_pcm_fromspan(xbus_t *xbus, xpd_t *xpd, xpp_line_t wanted_lines, xpacket_t *pack)
{
	byte		*pcm;
	struct zt_chan	*chans;
	unsigned long	flags;
	int		i;
	int		subunit;
	xpp_line_t	pcm_mask = 0;


	BUG_ON(!xbus);
	BUG_ON(!xpd);
	BUG_ON(!pack);
	pcm = RPACKET_FIELD(pack, GLOBAL, PCM_WRITE, pcm);
	for(subunit = 0; subunit < MAX_SUBUNIT; subunit++) {
		xpd_t		*tmp_xpd;

		tmp_xpd = xpd_byaddr(xbus, xpd->addr.unit, subunit);
		if(!tmp_xpd || !tmp_xpd->card_present)
			continue;
		spin_lock_irqsave(&tmp_xpd->lock, flags);
		chans = tmp_xpd->span.chans;
		for_each_line(tmp_xpd, i) {
			if(IS_SET(wanted_lines, i)) {
				if(SPAN_REGISTERED(tmp_xpd)) {
#ifdef	DEBUG_PCMTX
					if(pcmtx >= 0 && pcmtx_chan == i)
						memset((u_char *)pcm, pcmtx, ZT_CHUNKSIZE);
					else
#endif
						memcpy((u_char *)pcm, chans[i].writechunk, ZT_CHUNKSIZE);
					// fill_beep((u_char *)pcm, tmp_xpd->addr.subunit, 2);
				} else
					memset((u_char *)pcm, 0x7F, ZT_CHUNKSIZE);
				pcm += ZT_CHUNKSIZE;
			}
		}
		pcm_mask |= PCM_SHIFT(wanted_lines, subunit);
		XPD_COUNTER(tmp_xpd, PCM_WRITE)++;
		spin_unlock_irqrestore(&tmp_xpd->lock, flags);
	}
	RPACKET_FIELD(pack, GLOBAL, PCM_WRITE, lines) = pcm_mask;
}

static void BRI_card_pcm_tospan(xbus_t *xbus, xpd_t *xpd, xpacket_t *pack)
{
	byte		*pcm;
	xpp_line_t	pcm_mask;
	unsigned long	flags;
	int		subunit;
	int		i;

	/*
	 * Subunit 0 handle all other subunits
	 */
	if(xpd->addr.subunit != 0)
		return;
	if(!SPAN_REGISTERED(xpd))
		return;
	pcm = RPACKET_FIELD(pack, GLOBAL, PCM_READ, pcm);
	pcm_mask = RPACKET_FIELD(pack, GLOBAL, PCM_WRITE, lines);
	for(subunit = 0; subunit < MAX_SUBUNIT; subunit++, pcm_mask >>= SUBUNIT_PCM_SHIFT) {
		xpd_t		*tmp_xpd;

		if(!pcm_mask)
			break;	/* optimize */
		tmp_xpd = xpd_byaddr(xbus, xpd->addr.unit, subunit);
		if(!tmp_xpd || !tmp_xpd->card_present || !SPAN_REGISTERED(tmp_xpd))
			continue;
		spin_lock_irqsave(&tmp_xpd->lock, flags);
		for (i = 0; i < 2; i++) {
			xpp_line_t	tmp_mask = pcm_mask & (BIT(0) | BIT(1));
			volatile u_char	*r;

			if(IS_SET(tmp_mask, i)) {
				r = tmp_xpd->span.chans[i].readchunk;
				// memset((u_char *)r, 0x5A, ZT_CHUNKSIZE);	// DEBUG
				// fill_beep((u_char *)r, 1, 1);	// DEBUG: BEEP
				memcpy((u_char *)r, pcm, ZT_CHUNKSIZE);
				pcm += ZT_CHUNKSIZE;
			}
		}
		XPD_COUNTER(tmp_xpd, PCM_READ)++;
		spin_unlock_irqrestore(&tmp_xpd->lock, flags);
	}
}

/*---------------- BRI: HOST COMMANDS -------------------------------------*/

static /* 0x0F */ HOSTCMD(BRI, XPD_STATE, bool on)
{
	BUG_ON(!xpd);
	XPD_DBG(GENERAL, xpd, "%s\n", (on)?"ON":"OFF");
	xpd_activation(xpd, on);
	return 0;
}

static /* 0x0F */ HOSTCMD(BRI, RING, lineno_t chan, bool on)
{
	XPD_ERR(xpd, "%s: Unsupported\n", __FUNCTION__);
	return -ENOSYS;
}

static /* 0x0F */ HOSTCMD(BRI, RELAY_OUT, byte which, bool on)
{
	XPD_ERR(xpd, "%s: Unsupported\n", __FUNCTION__);
	return -ENOSYS;
}

static /* 0x33 */ HOSTCMD(BRI, SET_LED, enum bri_led_names which_led, enum led_state to_led_state)
{
	int			ret = 0;
	xframe_t		*xframe;
	xpacket_t		*pack;
	struct bri_leds		*bri_leds;
	struct BRI_priv_data	*priv;

	BUG_ON(!xbus);
	priv = xpd->priv;
	BUG_ON(!priv);
	XPD_DBG(LEDS, xpd, "%s -> %d\n",
		(which_led)?"RED":"GREEN",
		to_led_state);
	XFRAME_NEW_CMD(xframe, pack, xbus, BRI, SET_LED, xpd->xbus_idx);
	bri_leds = &RPACKET_FIELD(pack, BRI, SET_LED, bri_leds);
	bri_leds->state = to_led_state;
	bri_leds->led_sel = which_led;
	XPACKET_LEN(pack) = RPACKET_SIZE(BRI, SET_LED);
	ret = send_cmd_frame(xbus, xframe);
	priv->ledstate[which_led] = to_led_state;
	return ret;
}

static int write_state_register(xpd_t *xpd, byte value)
{
	int	ret;

	XPD_DBG(REGS, xpd, "value = 0x%02X\n", value);
	ret = xpp_register_request(xpd->xbus, xpd,
			0,		/* chipsel	*/
			1,		/* writing	*/
			0,		/* do_subreg	*/
			A_SU_WR_STA,	/* regnum	*/
			0,		/* subreg	*/
			value,		/* data_low	*/
			0		/* data_high	*/
			);
	return ret;
}

/*---------------- BRI: Astribank Reply Handlers --------------------------*/
static void su_new_state(xpd_t *xpd, byte reg_x30)
{
	xbus_t			*xbus;
	struct BRI_priv_data	*priv;
	su_rd_sta_t		new_state;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	xbus = xpd->xbus;
	if(!priv->initialized) {
		XPD_ERR(xpd, "%s called on uninitialized AB\n", __FUNCTION__);
		return;
	}
	new_state.reg = reg_x30;
	priv->reg30_ticks = 0;
	priv->reg30_good = 1;
	if((!IS_NT(xpd) && new_state.bits.v_su_sta == ST_TE_ACTIVATED) ||
		(IS_NT(xpd) && new_state.bits.v_su_sta == ST_NT_ACTIVATED)) {
		if(!priv->layer1_up) {
			layer1_state(xpd, 1);
			update_xpd_status(xpd, ZT_ALARM_NONE);
		}
	} else {
		/*
		 * Layer 1 disconnected
		 */
		if(priv->layer1_up) {
			layer1_state(xpd, 0);
			dchan_state(xpd, 0);
		}
		/*
		 * Do NOT notify Zaptel about the disconnection.
		 * If we do, Asterisk stops transmitting on the D-channel and
		 * we can't reactivate layer-1.
		 * Without the notification, Asterisk thinks that we are active
		 * (although the PSTN stopped layer-1) and on call setup, sends
		 * us D-channel data, which triggers the layer-1 activation.
		 */
#if 0
		update_xpd_status(xpd, ZT_ALARM_RED);
#endif
	}
	if (priv->state_register.bits.v_su_sta == new_state.bits.v_su_sta)
		return;	/* same same */
	DBG(SIGNAL, "%02X ---> %02X\n", priv->state_register.reg, reg_x30);
	XPD_DBG(SIGNAL, xpd, "%s%i\n", IS_NT(xpd)?"G":"F", new_state.bits.v_su_sta);

	if(!IS_NT(xpd)) {
		/* disable T3 ? */
		if ((new_state.bits.v_su_sta <= ST_TE_DEACTIVATED) || (new_state.bits.v_su_sta >= ST_TE_ACTIVATED)) {
			XPD_DBG(SIGNAL, xpd, "Disable T3 ?\n");
			priv->t3 = HFC_TIMER_OFF;
		}
		switch (new_state.bits.v_su_sta) {
			case ST_TE_DEACTIVATED:		/* F3 */
				XPD_DBG(SIGNAL, xpd, "State ST_TE_DEACTIVATED (F3)\n");
				if (test_and_clear_bit(HFC_L1_ACTIVATED, &priv->l1_flags))
					priv->t4 = HFC_TIMER_T4;
				break;
			case ST_TE_SIGWAIT:		/* F4	*/
				XPD_DBG(SIGNAL, xpd, "State ST_TE_SIGWAIT (F4)\n");
				break;
			case ST_TE_IDENT:		/* F5	*/
				XPD_DBG(SIGNAL, xpd, "State ST_TE_IDENT (F5)\n");
				break;
			case ST_TE_SYNCED:		/* F6	*/
				XPD_DBG(SIGNAL, xpd, "State ST_TE_SYNCED (F6)\n");
				break;
			case ST_TE_ACTIVATED:		/* F7 */
				XPD_DBG(SIGNAL, xpd, "State ST_TE_ACTIVATED (F7)\n");
				if (priv->t4 > HFC_TIMER_OFF)
					priv->t4 = HFC_TIMER_OFF;
				clear_bit(HFC_L1_ACTIVATING, &priv->l1_flags);
				set_bit(HFC_L1_ACTIVATED, &priv->l1_flags);
				update_xpd_status(xpd, ZT_ALARM_NONE);
				break;

			case ST_TE_LOST_FRAMING:	/* F8 */
				XPD_DBG(SIGNAL, xpd, "State ST_TE_LOST_FRAMING (F8)\n");
				priv->t4 = HFC_TIMER_OFF;
				break;
			default:
				XPD_NOTICE(xpd, "Bad TE state: %d\n", new_state.bits.v_su_sta);
				break;
		}

	} else if(IS_NT(xpd)) {
		switch (new_state.bits.v_su_sta) {
			case ST_NT_DEACTIVATED:		/* G1 */
				XPD_DBG(SIGNAL, xpd, "State ST_NT_DEACTIVATED (G1)\n");
				clear_bit(HFC_L1_ACTIVATED, &priv->l1_flags);
				priv->t1 = HFC_TIMER_OFF;
				break;
			case ST_NT_ACTIVATING:		/* G2 */
				XPD_DBG(SIGNAL, xpd, "State ST_NT_ACTIVATING (G2)\n");
				xpd_activation(xpd, 1);
				break;
			case ST_NT_ACTIVATED:		/* G3 */
				XPD_DBG(SIGNAL, xpd, "State ST_NT_ACTIVATED (G3)\n");
				clear_bit(HFC_L1_ACTIVATING, &priv->l1_flags);
				set_bit(HFC_L1_ACTIVATED, &priv->l1_flags);
				priv->t1 = HFC_TIMER_OFF;
				break;
			case ST_NT_DEACTIVTING:		/* G4 */
				XPD_DBG(SIGNAL, xpd, "State ST_NT_DEACTIVTING (G4)\n");
				priv->t1 = HFC_TIMER_OFF;
				break;
			default:
				XPD_NOTICE(xpd, "Bad NT state: %d\n", new_state.bits.v_su_sta);
				break;
		}
	} else
		XPD_ERR(xpd, "%s: Unknown xpd type %d\n", __FUNCTION__, xpd->type);
	priv->state_register.reg = new_state.reg;
}

static int BRI_card_register_reply(xbus_t *xbus, xpd_t *xpd, reg_cmd_t *info)
{
	unsigned long		flags;
	struct BRI_priv_data	*priv;
	int			ret;

	spin_lock_irqsave(&xpd->lock, flags);
	priv = xpd->priv;
	BUG_ON(!priv);
	if(REG_FIELD(info, do_subreg)) {
		XPD_DBG(REGS, xpd, "RS %02X %02X %02X\n",
				REG_FIELD(info, regnum), REG_FIELD(info, subreg), REG_FIELD(info, data_low));
	} else {
		if (REG_FIELD(info, regnum) != A_SU_RD_STA)
			XPD_DBG(REGS, xpd, "RD %02X %02X\n",
					REG_FIELD(info, regnum), REG_FIELD(info, data_low));
	}
	if(info->multibyte) {
		XPD_DBG(REGS, xpd, "Got Multibyte: %d bytes, eoframe: %d\n",
				info->bytes, info->eoframe);
		ret = rx_dchan(xpd, info);
		if (ret < 0) {
			priv->dchan_rx_drops++;
			if(atomic_read(&xpd->open_counter) > 0)
				XPD_NOTICE(xpd, "Multibyte Drop: errno=%d\n", ret);
		} 
		goto end;
	}
	if(REG_FIELD(info, regnum) == A_SU_RD_STA) {
		su_new_state(xpd, REG_FIELD(info, data_low));
	}

	/* Update /proc info only if reply relate to the last slic read request */
	if(
			REG_FIELD(&priv->requested_reply, regnum) == REG_FIELD(info, regnum) &&
			REG_FIELD(&priv->requested_reply, do_subreg) == REG_FIELD(info, do_subreg) &&
			REG_FIELD(&priv->requested_reply, subreg) == REG_FIELD(info, subreg)) {
		priv->last_reply = *info;
	}
	
end:
	spin_unlock_irqrestore(&xpd->lock, flags);
	return 0;
}

static xproto_table_t PROTO_TABLE(BRI_NT) = {
	.owner = THIS_MODULE,
	.entries = {
		/*	Table	Card	Opcode		*/
	},
	.name = "BRI_NT",
	.type = XPD_TYPE_BRI_NT,
	.xops = {
		.card_new	= BRI_card_new,
		.card_init	= BRI_card_init,
		.card_remove	= BRI_card_remove,
		.card_zaptel_preregistration	= BRI_card_zaptel_preregistration,
		.card_zaptel_postregistration	= BRI_card_zaptel_postregistration,
		.card_hooksig	= BRI_card_hooksig,
		.card_tick	= BRI_card_tick,
		.card_pcm_fromspan	= BRI_card_pcm_fromspan,
		.card_pcm_tospan	= BRI_card_pcm_tospan,
		.card_ioctl	= BRI_card_ioctl,
		.card_close	= BRI_card_close,
		.card_register_reply	= BRI_card_register_reply,

		.RING		= XPROTO_CALLER(BRI, RING),
		.RELAY_OUT	= XPROTO_CALLER(BRI, RELAY_OUT),
		.XPD_STATE	= XPROTO_CALLER(BRI, XPD_STATE),
	},
	.packet_is_valid = bri_packet_is_valid,
	.packet_dump = bri_packet_dump,
};

static xproto_table_t PROTO_TABLE(BRI_TE) = {
	.owner = THIS_MODULE,
	.entries = {
		/*	Table	Card	Opcode		*/
	},
	.name = "BRI_TE",
	.type = XPD_TYPE_BRI_TE,
	.xops = {
		.card_new	= BRI_card_new,
		.card_init	= BRI_card_init,
		.card_remove	= BRI_card_remove,
		.card_zaptel_preregistration	= BRI_card_zaptel_preregistration,
		.card_zaptel_postregistration	= BRI_card_zaptel_postregistration,
		.card_hooksig	= BRI_card_hooksig,
		.card_tick	= BRI_card_tick,
		.card_pcm_fromspan	= BRI_card_pcm_fromspan,
		.card_pcm_tospan	= BRI_card_pcm_tospan,
		.card_ioctl	= BRI_card_ioctl,
		.card_close	= BRI_card_close,
		.card_register_reply	= BRI_card_register_reply,

		.RING		= XPROTO_CALLER(BRI, RING),
		.RELAY_OUT	= XPROTO_CALLER(BRI, RELAY_OUT),
		.XPD_STATE	= XPROTO_CALLER(BRI, XPD_STATE),
	},
	.packet_is_valid = bri_packet_is_valid,
	.packet_dump = bri_packet_dump,
};

static bool bri_packet_is_valid(xpacket_t *pack)
{
	const xproto_entry_t	*xe_nt = NULL;
	const xproto_entry_t	*xe_te = NULL;
	// DBG(GENERAL, "\n");
	xe_nt = xproto_card_entry(&PROTO_TABLE(BRI_NT), XPACKET_OP(pack));
	xe_te = xproto_card_entry(&PROTO_TABLE(BRI_TE), XPACKET_OP(pack));
	return xe_nt != NULL || xe_te != NULL;
}

static void bri_packet_dump(const char *msg, xpacket_t *pack)
{
	DBG(GENERAL, "%s\n", msg);
}
/*------------------------- REGISTER Handling --------------------------*/

static int proc_bri_info_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int			len = 0;
	unsigned long		flags;
	xpd_t			*xpd = data;
	struct BRI_priv_data	*priv;

	DBG(PROC, "\n");
	if(!xpd)
		return -ENODEV;
	spin_lock_irqsave(&xpd->lock, flags);
	priv = xpd->priv;
	BUG_ON(!priv);
	len += sprintf(page + len, "%05d Layer 1: ", priv->poll_counter);
	if(priv->reg30_good) {
		len += sprintf(page + len, "%-5s ", (priv->layer1_up) ? "UP" : "DOWN");
		len += sprintf(page + len, "%c%d %-15s -- fr_sync=%d t2_exp=%d info0=%d g2_g3=%d\n",
					IS_NT(xpd)?'G':'F',
					priv->state_register.bits.v_su_sta,
					xhfc_state_name(xpd->type, priv->state_register.bits.v_su_sta),
					priv->state_register.bits.v_su_fr_sync,
					priv->state_register.bits.v_su_t2_exp,
					priv->state_register.bits.v_su_info0,
					priv->state_register.bits.v_g2_g3);
	} else
		len += sprintf(page + len, "Unkown\n");
	if(IS_NT(xpd))
		len += sprintf(page + len, "T1 Timer: %d\n", priv->t1);
	len += sprintf(page + len, "Tick Counter: %d\n", priv->tick_counter);
	len += sprintf(page + len, "Last Poll Reply: %d ticks ago\n", priv->reg30_ticks);
	len += sprintf(page + len, "reg30_good=%d\n", priv->reg30_good);
	len += sprintf(page + len, "D-Channel: TX=[%5d]    RX=[%5d]    BAD=[%5d] ",
			priv->dchan_tx_counter, priv->dchan_rx_counter, priv->dchan_rx_drops);
	if(priv->dchan_alive) {
		len += sprintf(page + len, "(alive %d K-ticks)\n",
			priv->dchan_alive_ticks/1000);
	} else {
		len += sprintf(page + len, "(dead)\n");
	}
	len += sprintf(page + len, "dchan_notx_ticks: %d\n", priv->dchan_notx_ticks);
	len += sprintf(page + len, "dchan_norx_ticks: %d\n", priv->dchan_norx_ticks);
	len += sprintf(page + len, "LED: %-10s = %d\n", "GREEN", priv->ledstate[GREEN_LED]);
	len += sprintf(page + len, "LED: %-10s = %d\n", "RED", priv->ledstate[RED_LED]);
	len += sprintf(page + len, "\nDCHAN:\n");
	len += sprintf(page + len, "\n");
	spin_unlock_irqrestore(&xpd->lock, flags);
	if (len <= off+count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

/*
 *
 * Direct/Indirect
 *     |
 *     | Reg#
 *     | |
 *     | |  Data (only in Write)
 *     | |    |
 *     | |  +-+-+
 *     v v  v   v
 * FF WD 06 01 05
 * ^  ^
 * |  |
 * |  Write/Read
 * |
 * Chan#
 *
 */
static int handle_register_command(xpd_t *xpd, char *cmdline)
{
	unsigned		chipsel;
	unsigned		data = 0;
	unsigned		xdata1 = 0;
	unsigned		xdata2 = 0;
	char			op;		/* [W]rite, [R]ead */
	char			reg_type;	/* [D]irect, [S]ubregister */
	int			reg_num;
	int			subreg;
	int			elements;
	bool			writing;
	char			*p;
	reg_cmd_t		regcmd;
	xbus_t			*xbus;
	int			ret = -EINVAL;
	struct BRI_priv_data	*priv;
	byte			buf[MAX_PROC_WRITE];

	BUG_ON(!xpd);
	xbus = xpd->xbus;
	BUG_ON(!xbus);
	priv = xpd->priv;
	BUG_ON(!priv);
	if((p = strchr(cmdline, '#')) != NULL)	/* Truncate comments */
		*p = '\0';
	if((p = strchr(cmdline, ';')) != NULL)	/* Truncate comments */
		*p = '\0';
	for(p = cmdline; *p && (*p == ' ' || *p == '\t'); p++) /* Trim leading whitespace */
		;
	if(*p == '\0')
		return 0;

	if(!XBUS_GET(xbus)) {
		XBUS_DBG(GENERAL, xbus, "Dropped packet. Is shutting down.\n");
		return -EBUSY;
	}
	memset(buf, 0, MAX_PROC_WRITE);
	elements = sscanf(cmdline, "%d %c%c %x %x %x %x %x",
			&chipsel,
			&op, &reg_type, &reg_num,
			&subreg,
			&data, &xdata1, &xdata2);
	XPD_DBG(PROC, xpd, "'%s': %d %c%c %02X %02X %02X\n", cmdline, chipsel, op, reg_type, reg_num, subreg, data);
	if(elements < 3) {	// At least: chipsel, op, reg_type, reg_num
		ERR("Not enough arguments: (%d args) '%s'\n", elements, cmdline);
		goto out;
	}
	if(!VALID_CHIPSEL(chipsel)) {
		ERR("Bad chip select number: %d\n", chipsel);
		goto out;
	}
	REG_FIELD(&regcmd, chipsel) = chipsel;
	switch(op) {
		case 'W':
			writing = 1;
			break;
		case 'R':
			writing = 0;
			break;
		default:
			ERR("Unkown operation type '%c'\n", op);
			goto out;
	}
	if(
			(op == 'W' && reg_type == 'D' && elements != 5) ||
			(op == 'W' && reg_type == 'S' && elements != 6) ||
			(op == 'R' && reg_type == 'D' && elements != 4) ||
			(op == 'R' && reg_type == 'S' && elements != 4)
	  ) {
		ERR("Bad number of elements: '%s' (%d elements): %d %c%c %02X %02X %02X\n",
				cmdline, elements,
				chipsel, op, reg_type, reg_num, subreg, data);
		goto out;
	}
	switch(reg_type) {
		case 'S':
			REG_FIELD(&regcmd, do_subreg) = 1;
			REG_FIELD(&regcmd, regnum) = reg_num;
			REG_FIELD(&regcmd, subreg) = subreg;
			REG_FIELD(&regcmd, data_low) = data;
			break;
		case 'D':
			REG_FIELD(&regcmd, do_subreg) = 0;
			REG_FIELD(&regcmd, regnum) = reg_num;
			REG_FIELD(&regcmd, subreg) = 0;
			REG_FIELD(&regcmd, data_low) = subreg;
			break;
		case 'M':	/* Multi without eoftx */
		case 'm':	/* Multi with eoftx */
			if(!writing) {
				ERR("Read multibyte is not implemented\n");
				goto out;
			}
			elements -= 3;
			REG_XDATA(&regcmd)[0] = reg_num;
			REG_XDATA(&regcmd)[1] = subreg;
			REG_XDATA(&regcmd)[2] = data;
			REG_XDATA(&regcmd)[3] = xdata1;
			REG_XDATA(&regcmd)[4] = xdata2;
			ret = send_bri_multibyte(xpd, REG_XDATA(&regcmd), elements, (reg_type == 'm'));
			goto out;
		default:
			ERR("Unkown register type '%c'\n", reg_type);
			goto out;
	}
	regcmd.bytes = sizeof(regcmd) - 1;
	REG_FIELD(&regcmd, read_request) = writing;
	REG_FIELD(&regcmd, data_high) = 0;
	priv->requested_reply = regcmd;
	if(print_dbg)
		dump_reg_cmd("BRI", &regcmd, 1);
	ret = xpp_register_request(xpd->xbus, xpd,
			REG_FIELD(&regcmd, chipsel),
			writing,
			REG_FIELD(&regcmd, do_subreg),
			REG_FIELD(&regcmd, regnum),
			REG_FIELD(&regcmd, subreg),
			REG_FIELD(&regcmd, data_low),
			REG_FIELD(&regcmd, data_high));
out:
	XBUS_PUT(xbus);
	return ret;
}

static int proc_xpd_register_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	xpd_t		*xpd = data;
	char		buf[MAX_PROC_WRITE];
	char		*p;
	int		i;
	int		ret;

	if(!xpd)
		return -ENODEV;
	for(i = 0; i < count; /* noop */) {
		for(p = buf; p < buf + MAX_PROC_WRITE; p++) {	/* read a line */
			if(i >= count)
				break;
			if(get_user(*p, buffer + i))
				return -EFAULT;
			i++;
			if(*p == '\n' || *p == '\r')	/* whatever */
				break;
		}
		if(p >= buf + MAX_PROC_WRITE)
			return -E2BIG;
		*p = '\0';
		ret = handle_register_command(xpd, buf);
		if(ret < 0)
			return ret;
		msleep(1);
	}
	return count;
}


static int proc_xpd_register_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int			len = 0;
	unsigned long		flags;
	xpd_t			*xpd = data;
	reg_cmd_t		*info;
	struct BRI_priv_data	*priv;

	if(!xpd)
		return -ENODEV;
	priv = xpd->priv;
	BUG_ON(!priv);
	spin_lock_irqsave(&xpd->lock, flags);
	info = &priv->last_reply;
	len += sprintf(page + len, "# Writing bad data into this file may damage your hardware!\n");
	len += sprintf(page + len, "# Consult firmware docs first\n");
	len += sprintf(page + len, "#\n");
	if(REG_FIELD(info, do_subreg)) {
		len += sprintf(page + len, "#CH\tOP\tReg.\tSub\tDL\n");
		len += sprintf(page + len, "%2d\tRS\t%02X\t%02X\t%02X\n",
				REG_FIELD(info, chipsel),
				REG_FIELD(info, regnum), REG_FIELD(info, subreg), REG_FIELD(info, data_low));
	} else {
		len += sprintf(page + len, "#CH\tOP\tReg.\tDL\n");
		len += sprintf(page + len, "%2d\tRD\t%02X\t%02X\n",
				REG_FIELD(info, chipsel),
				REG_FIELD(info, regnum), REG_FIELD(info, data_low));
	}
	spin_unlock_irqrestore(&xpd->lock, flags);
	if (len <= off+count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

int __init card_bri_startup(void)
{
	DBG(GENERAL, "\n");

	INFO("revision %s\n", XPP_VERSION);
	xproto_register(&PROTO_TABLE(BRI_NT));
	xproto_register(&PROTO_TABLE(BRI_TE));
	return 0;
}

void __exit card_bri_cleanup(void)
{
	DBG(GENERAL, "\n");
	xproto_unregister(&PROTO_TABLE(BRI_NT));
	xproto_unregister(&PROTO_TABLE(BRI_TE));
}

MODULE_DESCRIPTION("XPP BRI Card Driver");
MODULE_AUTHOR("Oron Peled <oron@actcom.co.il>");
MODULE_LICENSE("GPL");
MODULE_VERSION(XPP_VERSION);
MODULE_ALIAS_XPD(XPD_TYPE_BRI_NT);
MODULE_ALIAS_XPD(XPD_TYPE_BRI_TE);

module_init(card_bri_startup);
module_exit(card_bri_cleanup);
