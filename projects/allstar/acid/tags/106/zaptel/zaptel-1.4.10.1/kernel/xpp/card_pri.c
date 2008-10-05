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
#include "card_pri.h"
#include "zap_debug.h"
#include "xpd.h"
#include "xbus-core.h"

static const char rcsid[] = "$Id: card_pri.c 4039 2008-03-21 01:51:39Z tzafrir $";

DEF_PARM(int, print_dbg, 0, 0644, "Print DBG statements");	/* must be before zap_debug.h */
DEF_PARM(uint, poll_interval, 500, 0644, "Poll channel state interval in milliseconds (0 - disable)");

#define	PRI_LINES_BITMASK	BITMASK(31)
#define	PRI_DCHAN_SIGCAP	(			  \
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
#define	PRI_BCHAN_SIGCAP	ZT_SIG_CLEAR
#define	MAX_SLAVES		4		/* we have MUX of 4 clocks */


/*---------------- PRI Protocol Commands ----------------------------------*/

static bool pri_packet_is_valid(xpacket_t *pack);
static void pri_packet_dump(const char *msg, xpacket_t *pack);
static int proc_pri_info_read(char *page, char **start, off_t off, int count, int *eof, void *data);
static int proc_pri_info_write(struct file *file, const char __user *buffer, unsigned long count, void *data);
static int proc_xpd_register_read(char *page, char **start, off_t off, int count, int *eof, void *data);
static int proc_xpd_register_write(struct file *file, const char __user *buffer, unsigned long count, void *data);
static int pri_startup(struct zt_span *span);
static int pri_shutdown(struct zt_span *span);

#define	PROC_REGISTER_FNAME	"slics"
#define	PROC_PRI_INFO_FNAME	"pri_info"

#define	VALID_CHIPSEL(x)	((x) == 0)

enum pri_protocol {
	PRI_PROTO_0  	= 0,
	PRI_PROTO_E1	= 1,
	PRI_PROTO_T1 	= 2,
	PRI_PROTO_J1 	= 3
};

static const char *pri_protocol_name(enum pri_protocol pri_protocol)
{
	static const char *protocol_names[] = {
		[PRI_PROTO_0] = "??",	/* unkown */
		[PRI_PROTO_E1] = "E1",
		[PRI_PROTO_T1] = "T1",
		[PRI_PROTO_J1] = "J1"
		};
	return protocol_names[pri_protocol];
}

static int pri_num_channels(enum pri_protocol pri_protocol)
{
	static int num_channels[] = {
		[PRI_PROTO_0] = 0,
		[PRI_PROTO_E1] = 31,
		[PRI_PROTO_T1] = 24,
		[PRI_PROTO_J1] = 0
		};
	return num_channels[pri_protocol];
}

static const char *type_name(enum pri_protocol pri_protocol, bool is_nt)
{
	static const char	*names[2][4] = {
		/* TE */ [0]	= {
				[PRI_PROTO_0] = "Unknown_TE",
				[PRI_PROTO_E1] = "E1_TE",
				[PRI_PROTO_T1] = "T1_TE",
				[PRI_PROTO_J1] = "J1_TE"
			},
		/* NT */ [1]	= {
				[PRI_PROTO_0] = "Unknown_NT",
				[PRI_PROTO_E1] = "E1_NT",
				[PRI_PROTO_T1] = "T1_NT",
				[PRI_PROTO_J1] = "J1_NT"
			}
		};
	int	term = (is_nt) ? 1 : 0;

	return names[term][pri_protocol];
}

static int pri_linecompat(enum pri_protocol pri_protocol)
{
	static const int linecompat[] = {
		[PRI_PROTO_0] = 0,
		[PRI_PROTO_E1] =
			/* coding */
			ZT_CONFIG_CCS |
			// CAS |
			ZT_CONFIG_CRC4 |
			/* framing */
			ZT_CONFIG_AMI | ZT_CONFIG_HDB3,
		[PRI_PROTO_T1] =
			/* coding */
			// ZT_CONFIG_D4 |
			ZT_CONFIG_ESF |
			/* framing */
			ZT_CONFIG_AMI | ZT_CONFIG_B8ZS,
		[PRI_PROTO_J1] = 0
	};

	DBG(GENERAL, "pri_linecompat: pri_protocol=%d\n", pri_protocol);
	return linecompat[pri_protocol];
}

#define	PRI_DCHAN_IDX(priv)	((priv)->dchan_num - 1)

enum pri_led_state {
	PRI_LED_OFF		= 0x0,
	PRI_LED_ON		= 0x1,
	/*
	 * We blink by software from driver, so that
	 * if the driver malfunction that blink would stop.
	 */
	// PRI_LED_BLINK_SLOW	= 0x2,	/* 1/2 a second blink cycle */
	// PRI_LED_BLINK_FAST	= 0x3	/* 1/4 a second blink cycle */
};

enum pri_led_selectors {
	TE_RED_LED	= 0,
	TE_GREEN_LED	= 1,
	NT_RED_LED	= 2,
	NT_GREEN_LED	= 3,
};

#define	NUM_LEDS	4

struct pri_leds {
	byte	state:2;	/* enum pri_led_state */
	byte	led_sel:2;	/* enum pri_led_selectors */
	byte	reserved:4;
};

#define	REG_FRS0	0x4C	/* Framer Receive Status Register 0 */
#define	REG_FRS0_LMFA	BIT(1)	/* Loss of Multiframe Alignment */
#define	REG_FRS0_NMF	BIT(2)	/* No Multiframe Alignment Found */
#define	REG_FRS0_RRA	BIT(4)	/* Receive Remote Alarm: T1-YELLOW-Alarm */
#define	REG_FRS0_LFA	BIT(5)	/* Loss of Frame Alignment */
#define	REG_FRS0_AIS	BIT(6)	/* Alarm Indication Signal: T1-BLUE-Alarm */
#define	REG_FRS0_LOS	BIT(7)	/* Los Of Signal: T1-RED-Alarm */

#define	REG_FRS1	0x4D	/* Framer Receive Status Register 1 */

#define	REG_LIM0	0x36
#define	REG_LIM0_MAS	BIT(0)	/* Master Mode, DCO-R circuitry is frequency
                                                                                          synchronized to the clock supplied by SYNC */
#define	REG_LIM0_RTRS	BIT(5)	/*
				 * Receive Termination Resistance Selection:
				 * integrated resistor to create 75 Ohm termination (100 || 300 = 75)
				 * 0 = 100 Ohm
				 * 1 = 75 Ohm
				 */
#define	REG_LIM0_LL	BIT(1)	/* LL (Local Loopback) */

#define	REG_FMR0	0x1C
#define	REG_FMR0_E_RC0	BIT(4)	/* Receive Code - LSB */
#define	REG_FMR0_E_RC1	BIT(5)	/* Receive Code - MSB */
#define	REG_FMR0_E_XC0	BIT(6)	/* Transmit Code - LSB */
#define	REG_FMR0_E_XC1	BIT(7)	/* Transmit Code - MSB */

#define	REG_FMR1	0x1D
#define	REG_FMR1_XAIS	BIT(0)	/* Transmit AIS toward transmit end */
#define	REG_FMR1_SSD0	BIT(1)
#define	REG_FMR1_ECM	BIT(2)
#define	REG_FMR1_XFS	BIT(3)
#define	REG_FMR1_PMOD	BIT(4)	/* E1 = 0, T1/J1 = 1 */
#define	REG_FMR1_EDL	BIT(5)
#define	REG_FMR1_AFR	BIT(6)

#define	REG_FMR2	0x1E
#define	REG_FMR2_E_ALMF	BIT(0)	/* Automatic Loss of Multiframe */
#define	REG_FMR2_T_EXZE	BIT(0)	/* Excessive Zeros Detection Enable */
#define	REG_FMR2_E_AXRA	BIT(1)	/* Automatic Transmit Remote Alarm */
#define	REG_FMR2_T_AXRA	BIT(1)	/* Automatic Transmit Remote Alarm */
#define	REG_FMR2_E_PLB	BIT(2)	/* Payload Loop-Back */
#define	REG_FMR2_E_RFS0	BIT(6)	/* Receive Framing Select - LSB */
#define	REG_FMR2_E_RFS1	BIT(7)	/* Receive Framing Select - MSB */
#define	REG_FMR2_T_SSP	BIT(5)	/* Select Synchronization/Resynchronization Procedure */
#define	REG_FMR2_T_MCSP	BIT(6)	/* Multiple Candidates Synchronization Procedure */
#define	REG_FMR2_T_AFRS	BIT(7)	/* Automatic Force Resynchronization */

#define	REG_FMR4	0x20
#define	REG_FMR4_FM1	BIT(1)

#define REG_XSP_E	0x21
#define REG_FMR5_T	0x21
#define	REG_XSP_E_XSIF	BIT(2)	/* Transmit Spare Bit For International Use (FAS Word)  */
#define	REG_FMR5_T_XTM	BIT(2)	/* Transmit Transparent Mode  */
#define	REG_XSP_E_AXS	BIT(3)	/* Automatic Transmission of Submultiframe Status  */
#define	REG_XSP_E_EBP	BIT(4)	/* E-Bit Polarity, Si-bit position of every outgoing CRC multiframe  */
#define	REG_XSP_E_CASEN	BIT(7)	/* Channel Associated Signaling Enable  */

#define	REG_RC0		0x24
#define	REG_RC0_SJR	BIT(7)	/* T1 = 0, J1 = 1 */

#define	REG_CMR1	0x44
#define	REG_CMR1_DRSS	(BIT(7) | BIT(6))
#define	REG_CMR1_RS	(BIT(5) | BIT(4))
#define	REG_CMR1_STF	BIT(2)

struct PRI_priv_data {
	bool				is_nt;
	bool				clock_source;
	struct proc_dir_entry		*regfile;
	struct proc_dir_entry		*pri_info;
	enum pri_protocol		pri_protocol;
	int				deflaw;
	unsigned int			dchan_num;
	bool				initialized;
	bool				local_loopback;
	reg_cmd_t			requested_reply;
	reg_cmd_t			last_reply;
	uint				poll_noreplies;
	uint				layer1_replies;
	byte				reg_frs0;
	byte				reg_frs1;
	bool				layer1_up;
	int				alarms;
	byte				dchan_tx_sample;
	byte				dchan_rx_sample;
	uint				dchan_tx_counter;
	uint				dchan_rx_counter;
	bool				dchan_alive;
	uint				dchan_alive_ticks;
	enum pri_led_state		ledstate[NUM_LEDS];
};

static xproto_table_t	PROTO_TABLE(PRI);

DEF_RPACKET_DATA(PRI, SET_LED,	/* Set one of the LED's */
	struct pri_leds		pri_leds;
	);


static /* 0x33 */ DECLARE_CMD(PRI, SET_LED, enum pri_led_selectors led_sel, enum pri_led_state to_led_state);

#define	DO_LED(xpd, which, tostate)	\
		CALL_PROTO(PRI, SET_LED, (xpd)->xbus, (xpd), (which), (tostate))

/*---------------- PRI: Methods -------------------------------------------*/

static int query_subunit(xpd_t *xpd, byte regnum)
{
#if 0
	XPD_DBG(GENERAL, xpd, "(%d%d): REG=0x%02X\n",
		xpd->addr.unit, xpd->addr.subunit,
		regnum);
#endif
	return xpp_register_request(
			xpd->xbus, xpd,
			0,			/* chipsel */
			0,			/* writing */
			1,			/* do_subreg */
			regnum,
			xpd->addr.subunit,	/* subreg */
			0,			/* data_L */
			0);			/* data_H */
}


static int write_subunit(xpd_t *xpd, byte regnum, byte val)
{
	XPD_DBG(REGS, xpd, "(%d%d): REG=0x%02X dataL=0x%02X\n",
		xpd->addr.unit, xpd->addr.subunit,
		regnum, val);
	return xpp_register_request(
			xpd->xbus, xpd,
			0,			/* chipsel */
			1,			/* writing */
			1,			/* do_subreg */
			regnum,
			xpd->addr.subunit,	/* subreg */
			val,			/* data_L */
			0);			/* data_H */
}

static int pri_write_reg(xpd_t *xpd, int regnum, byte val)
{
	XPD_DBG(REGS, xpd, "(%d%d): REG=0x%02X dataL=0x%02X\n",
		xpd->addr.unit, xpd->addr.subunit,
		regnum, val);
	return xpp_register_request(
			xpd->xbus, xpd,
			0,		/* chipsel */
			1,		/* writing */
			0,		/* do_subreg */
			regnum,
			0,		/* subreg */
			val,	/* data_L */
			0);		/* data_H */
}

static xpd_t *PRI_card_new(xbus_t *xbus, int unit, int subunit, const xproto_table_t *proto_table, byte subtype, byte revision)
{
	xpd_t			*xpd = NULL;
	struct PRI_priv_data	*priv;
	int			channels = min(31, CHANNELS_PERXPD);	/* worst case */

	XBUS_DBG(GENERAL, xbus, "\n");
	xpd = xpd_alloc(sizeof(struct PRI_priv_data), proto_table, channels);
	if(!xpd)
		return NULL;
	priv = xpd->priv;
	xpd->revision = revision;
	priv->pri_protocol = PRI_PROTO_0;	/* Default, changes in set_pri_proto() */
	priv->deflaw = ZT_LAW_DEFAULT;		/* Default, changes in set_pri_proto() */
	xpd->type_name =
		type_name(priv->pri_protocol, 0);	/* Default, changes in set_nt() */
	return xpd;
}

static void clean_proc(xbus_t *xbus, xpd_t *xpd)
{
	struct PRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	XPD_DBG(PROC, xpd, "\n");
#ifdef	CONFIG_PROC_FS
	if(priv->regfile) {
		XPD_DBG(PROC, xpd, "Removing registers file\n");
		priv->regfile->data = NULL;
		remove_proc_entry(PROC_REGISTER_FNAME, xpd->proc_xpd_dir);
	}
	if(priv->pri_info) {
		XPD_DBG(PROC, xpd, "Removing xpd PRI_INFO file\n");
		remove_proc_entry(PROC_PRI_INFO_FNAME, xpd->proc_xpd_dir);
	}
#endif
}

static bool valid_pri_modes(const xpd_t *xpd)
{
	struct PRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	if(
		priv->pri_protocol != PRI_PROTO_E1 &&
		priv->pri_protocol != PRI_PROTO_T1 &&
		priv->pri_protocol != PRI_PROTO_J1)
		return 0;
	return 1;
}

/*
 * Set E1/T1/J1
 * May only be called on unregistered xpd's
 * (the span and channel description are set according to this)
 */
static int set_pri_proto(xpd_t *xpd, enum pri_protocol set_proto)
{
	struct PRI_priv_data	*priv;
	int			deflaw;
	unsigned int		dchan_num;
	byte			fmr1 =
					REG_FMR1_AFR |
					REG_FMR1_XFS |
					REG_FMR1_ECM;
	byte			rc0 = 0;	/* FIXME: PCM offsets */

	BUG_ON(!xpd);
	priv = xpd->priv;
	if(SPAN_REGISTERED(xpd)) {
		XPD_NOTICE(xpd, "Registered as span %d. Cannot do setup pri protocol (%s)\n",
			xpd->span.spanno, __FUNCTION__);
		return -EBUSY;
	}
	switch(set_proto) {
		case PRI_PROTO_E1:
			deflaw = ZT_LAW_ALAW;
			dchan_num = 16;
			break;
		case PRI_PROTO_T1:
			deflaw = ZT_LAW_MULAW;
			dchan_num = 24;
			fmr1 |= REG_FMR1_PMOD;
			break;
		case PRI_PROTO_J1:
			/*
			 * Check all assumptions
			 */
			deflaw = ZT_LAW_MULAW;
			dchan_num = 24;
			fmr1 |= REG_FMR1_PMOD;
			rc0 |= REG_RC0_SJR;
			XPD_NOTICE(xpd, "J1 is not supported yet\n");
			return -ENOSYS;
		default:
			XPD_ERR(xpd, "%s: Unknown pri protocol = %d\n",
				__FUNCTION__, set_proto);
			return -EINVAL;
	}
	priv->pri_protocol = set_proto;
	xpd->channels = pri_num_channels(set_proto);
	xpd->pcm_len = RPACKET_HEADERSIZE + sizeof(xpp_line_t)  +  xpd->channels * ZT_CHUNKSIZE;
	xpd->wanted_pcm_mask = BITMASK(xpd->channels);
	priv->deflaw = deflaw;
	priv->dchan_num = dchan_num;
	xpd->type_name = type_name(priv->pri_protocol, priv->is_nt);
	XPD_DBG(GENERAL, xpd, "%s, channels=%d, dchan_num=%d, deflaw=%d\n",
			pri_protocol_name(set_proto),
			xpd->channels,
			priv->dchan_num,
			priv->deflaw
			);
	write_subunit(xpd, REG_FMR1, fmr1);
#ifdef JAPANEZE_SUPPORT
	if(rc0)
		write_subunit(xpd, REG_RC0, rc0);
#endif
	return 0;
}

static void zap_update_syncsrc(xpd_t *xpd)
{
	struct PRI_priv_data	*priv;
	xpd_t			*subxpd;
	int			best_spanno = 0;
	int			i;

	if(!SPAN_REGISTERED(xpd))
		return;
	for(i = 0; i < MAX_SLAVES; i++) {
		subxpd = xpd_byaddr(xpd->xbus, xpd->addr.unit, i);
		if(!subxpd)
			continue;
		priv = subxpd->priv;
		if(priv->clock_source) {
			if(best_spanno)
				XPD_ERR(xpd, "Duplicate XPD's with clock_source=1\n");
			best_spanno = subxpd->span.spanno;
		}
	}
	for(i = 0; i < MAX_SLAVES; i++) {
		subxpd = xpd_byaddr(xpd->xbus, xpd->addr.unit, i);
		if(!subxpd)
			continue;
		if(subxpd->span.syncsrc == best_spanno)
			XPD_DBG(SYNC, xpd, "Setting SyncSource to span %d\n", best_spanno);
		else
			XPD_DBG(SYNC, xpd, "Slaving to span %d\n", best_spanno);
		subxpd->span.syncsrc = best_spanno;
	}
}

/*
 * Called from:
 *   - set_master_mode() --
 *       As a result of ztcfg or writing to /proc/xpp/XBUS-??/XPD-/??/pri_info
 *   - layer1_state() --
 *       As a result of an alarm.
 */
static void set_clocking(xpd_t *xpd)
{
	xbus_t			*xbus;
	xpd_t			*best_xpd = NULL;
	int			best_subunit = -1;	/* invalid */
	int			best_subunit_prio = 0;
	int			i;

	xbus = get_xbus(xpd->xbus->num);
	/* Find subunit with best timing priority */
	for(i = 0; i < MAX_SLAVES; i++) {
		struct PRI_priv_data	*priv;
		xpd_t			*subxpd;
		
		subxpd = xpd_byaddr(xbus, xpd->addr.unit, i);
		if(!subxpd)
			continue;
		priv = subxpd->priv;
		if(priv->alarms != 0)
			continue;
		if(subxpd->timing_priority > best_subunit_prio) {
			best_xpd = subxpd;
			best_subunit = i;
			best_subunit_prio = subxpd->timing_priority;
		}
	}
	/* Now set it */
	if(best_xpd && ((struct PRI_priv_data *)(best_xpd->priv))->clock_source == 0) {
		byte	cmr1_val =
				REG_CMR1_RS |
				REG_CMR1_STF |
				(REG_CMR1_DRSS & (best_subunit << 6));
		XPD_DBG(SYNC, best_xpd,
			"ClockSource Set: cmr1=0x%02X\n", cmr1_val);
		pri_write_reg(xpd, REG_CMR1, cmr1_val);
		((struct PRI_priv_data *)(best_xpd->priv))->clock_source = 1;
	}
	/* clear old clock sources */
	for(i = 0; i < MAX_SLAVES; i++) {
		struct PRI_priv_data	*priv;
		xpd_t			*subxpd;

		subxpd = xpd_byaddr(xbus, xpd->addr.unit, i);
		if(subxpd && subxpd != best_xpd) {
			XPD_DBG(SYNC, subxpd, "Clearing clock source\n");
			priv = subxpd->priv;
			priv->clock_source = 0;
		}
	}
	zap_update_syncsrc(xpd);
	put_xbus(xbus);
}

/*
 * Normally set by the timing parameter in zaptel.conf
 * If this is called by ztcfg, than it's too late to change
 * zaptel sync priority (we are already registered)
 * There are two workarounds to mitigate this problem:
 * 1. So we set *our* sync master at least.
 * 2. And we try to call it with a sane default from set_nt()
 *    which is called before zaptel registration.
 */
static int set_master_mode(const char *msg, xpd_t *xpd)
{
	struct PRI_priv_data	*priv;
	byte			lim0 = 0;
	byte			xsp  = 0;
	bool			is_master_mode = xpd->timing_priority == 0;

	BUG_ON(!xpd);
	priv = xpd->priv;
	lim0 |= (priv->local_loopback) ? REG_LIM0_LL : 0;
	if(is_master_mode)
		lim0 |=  REG_LIM0_MAS;
	else
		lim0 &= ~REG_LIM0_MAS;
	if(priv->pri_protocol == PRI_PROTO_E1)
	{
		lim0 |= REG_LIM0_RTRS; /*  Receive termination: Integrated resistor is switched on (100 Ohm || 300 Ohm = 75 Ohm) */
		xsp  |= REG_XSP_E_EBP | REG_XSP_E_AXS | REG_XSP_E_XSIF;
	} else if(priv->pri_protocol == PRI_PROTO_T1) { 
		lim0 &= ~REG_LIM0_RTRS; /*  Receive termination: Integrated resistor is switched off (100 Ohm, no internal 300 Ohm)  */;
		xsp  |=  REG_FMR5_T_XTM;
	}
	XPD_DBG(SIGNAL, xpd, "%s(%s): %s\n", __FUNCTION__, msg, (is_master_mode) ? "MASTER" : "SLAVE");
	write_subunit(xpd, REG_LIM0 , lim0);
	write_subunit(xpd, REG_XSP_E, xsp);
	set_clocking(xpd);
	return 0;
}

static int set_nt(const char *msg, xpd_t *xpd, bool is_nt)
{
	struct PRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	if(SPAN_REGISTERED(xpd)) {
		XPD_NOTICE(xpd, "Registered as span %d. Cannot do %s(%s)\n",
			xpd->span.spanno, __FUNCTION__, msg);
		return -EBUSY;
	}
	priv->is_nt = is_nt;
	xpd->type_name = type_name(priv->pri_protocol, is_nt);
	xpd->direction = (is_nt) ? TO_PHONE : TO_PSTN;
	XPD_DBG(SIGNAL, xpd, "%s(%s): %s %s\n", __FUNCTION__, msg, xpd->type_name, (is_nt) ? "NT" : "TE");
	if(xpd->timing_priority == 0 && !is_nt) /* by default set timing priority from NT/TE */
		xpd->timing_priority = 1;
	set_master_mode(msg, xpd);
	return 0;
}

static int set_localloop(const char *msg, xpd_t *xpd, bool localloop)
{
	struct PRI_priv_data	*priv;
	byte			lim0 = 0;
	byte			xsp  = 0;

	BUG_ON(!xpd);
	priv = xpd->priv;
	if(SPAN_REGISTERED(xpd)) {
		XPD_NOTICE(xpd, "Registered as span %d. Cannot do %s(%s)\n",
			xpd->span.spanno, __FUNCTION__, msg);
		return -EBUSY;
	}
	lim0 |= (localloop) ? REG_LIM0_LL : 0;
	if(priv->is_nt)
		lim0 |=  REG_LIM0_MAS;
	else
		lim0 &= ~REG_LIM0_MAS;
	if(priv->pri_protocol == PRI_PROTO_E1)
	{
		lim0 |= REG_LIM0_RTRS; /*  Receive termination: Integrated resistor is switched on (100 Ohm || 300 Ohm = 75 Ohm) */
		xsp  |= REG_XSP_E_EBP | REG_XSP_E_AXS | REG_XSP_E_XSIF;
	} else if(priv->pri_protocol == PRI_PROTO_T1) { 
		lim0 &= ~REG_LIM0_RTRS ; /*  Receive termination: Integrated resistor is switched off (100 Ohm, no internal 300 Ohm)  */;
		xsp  |=  REG_FMR5_T_XTM;
	}
	priv->local_loopback = localloop;
	XPD_DBG(SIGNAL, xpd, "%s(%s): %s\n", __FUNCTION__, msg, (localloop) ? "LOCALLOOP" : "NO");
	write_subunit(xpd, REG_LIM0 , lim0);
	write_subunit(xpd, REG_XSP_E, xsp);
	return 0;
}

#define	VALID_CONFIG(bit,flg,str)	[bit] = { .flags = flg, .name = str }

static const struct {
	const char	*name;
	const int	flags;
} valid_spanconfigs[sizeof(unsigned int)*8] = {
	/* These apply to T1 */
//	VALID_CONFIG(4, ZT_CONFIG_D4, "D4"),	FIXME: should support
	VALID_CONFIG(5, ZT_CONFIG_ESF, "ESF"),
	VALID_CONFIG(6, ZT_CONFIG_AMI, "AMI"),
	VALID_CONFIG(7, ZT_CONFIG_B8ZS, "B8ZS"),
	/* These apply to E1 */
	VALID_CONFIG(8, ZT_CONFIG_CCS, "CCS"),
	VALID_CONFIG(9, ZT_CONFIG_HDB3, "HDB3"),
	VALID_CONFIG(10, ZT_CONFIG_CRC4, "CRC4"),
};

static int pri_lineconfig(xpd_t *xpd, int lineconfig)
{
	struct PRI_priv_data	*priv;
	const char		*framingstr = "";
	const char		*codingstr = "";
	const char		*crcstr = "";
	byte			fmr0 = 0;  /* Dummy initilizations to    */
	byte			fmr2 = 0;  /* silense false gcc warnings */
	byte			fmr4 = 0x0C;
	unsigned int		bad_bits;
	int			i;

	BUG_ON(!xpd);
	priv = xpd->priv;
	/*
	 * validate
	 */
	bad_bits = lineconfig & pri_linecompat(priv->pri_protocol);
	bad_bits = bad_bits ^ lineconfig;
	for(i = 0; i < ARRAY_SIZE(valid_spanconfigs); i++) {
		unsigned int	flags = valid_spanconfigs[i].flags;

		if(bad_bits & BIT(i)) {
			if(flags) {
				XPD_ERR(xpd,
					"Bad config item '%s' for %s. Ignore\n",
					valid_spanconfigs[i].name,
					pri_protocol_name(priv->pri_protocol));
			} else {
				/* we got real garbage */
				XPD_ERR(xpd,
					"Unknown config item 0x%lX for %s. Ignore\n",
					BIT(i),
					pri_protocol_name(priv->pri_protocol));
			}
		}
		if(flags && flags != BIT(i)) {
			ERR("%s: BUG: i=%d flags=0x%X\n",
				__FUNCTION__, i, flags);
			// BUG();
		}
	}
	if(bad_bits)
		goto bad_lineconfig;
	if(priv->pri_protocol == PRI_PROTO_E1)
		fmr2 = REG_FMR2_E_AXRA | REG_FMR2_E_ALMF;	/* 0x03 */
	else if(priv->pri_protocol == PRI_PROTO_T1)
		fmr2 = REG_FMR2_T_SSP | REG_FMR2_T_AXRA;	/* 0x22 */
	else if(priv->pri_protocol == PRI_PROTO_J1) {
		XPD_ERR(xpd, "J1 unsupported yet\n");
		return -ENOSYS;
	}
	if(priv->local_loopback)
		fmr2 |= REG_FMR2_E_PLB;
	/* framing first */
	if (lineconfig & ZT_CONFIG_B8ZS) {
		framingstr = "B8ZS";
		fmr0 = REG_FMR0_E_XC1 | REG_FMR0_E_XC0 | REG_FMR0_E_RC1 | REG_FMR0_E_RC0;
	} else if (lineconfig & ZT_CONFIG_AMI) {
		framingstr = "AMI";
		fmr0 = REG_FMR0_E_XC1 | REG_FMR0_E_RC1;
	} else if (lineconfig & ZT_CONFIG_HDB3) {
		framingstr = "HDB3";
		fmr0 = REG_FMR0_E_XC1 | REG_FMR0_E_XC0 | REG_FMR0_E_RC1 | REG_FMR0_E_RC0;
	}
	/* then coding */
	if (lineconfig & ZT_CONFIG_ESF) {
		codingstr = "ESF";
		fmr4 |= REG_FMR4_FM1;
		fmr2 |= REG_FMR2_T_AXRA | REG_FMR2_T_MCSP | REG_FMR2_T_SSP;
	} else if (lineconfig & ZT_CONFIG_D4) {
		codingstr = "D4";
	} else if (lineconfig & ZT_CONFIG_CCS) {
		codingstr = "CCS";
		/* do nothing */
	}
	/* E1's can enable CRC checking */
	if (lineconfig & ZT_CONFIG_CRC4) {
		crcstr = "CRC4";
		fmr2 |= REG_FMR2_E_RFS1;
	}
	XPD_DBG(GENERAL, xpd, "[%s] lineconfig=%s/%s/%s %s (0x%X)\n",
		(priv->is_nt)?"NT":"TE",
		framingstr, codingstr, crcstr,
		(lineconfig & ZT_CONFIG_NOTOPEN)?"YELLOW":"",
		lineconfig);
	if(fmr0 != 0) {
		XPD_DBG(GENERAL, xpd, "%s: fmr0(0x%02X) = 0x%02X\n", __FUNCTION__, REG_FMR0, fmr0);
		write_subunit(xpd, REG_FMR0, fmr0);
	}
	XPD_DBG(GENERAL, xpd, "%s: fmr4(0x%02X) = 0x%02X\n", __FUNCTION__, REG_FMR4, fmr4);
	write_subunit(xpd, REG_FMR4, fmr4);
	XPD_DBG(GENERAL, xpd, "%s: fmr2(0x%02X) = 0x%02X\n", __FUNCTION__, REG_FMR2, fmr2);
	write_subunit(xpd, REG_FMR2, fmr2);
	return 0;
bad_lineconfig:
	XPD_ERR(xpd, "Bad lineconfig. Abort\n");
	return -EINVAL;
}

/*
 * Called only for 'span' keyword in /etc/zaptel.conf
 */

static int pri_spanconfig(struct zt_span *span, struct zt_lineconfig *lc)
{
	xpd_t			*xpd = span->pvt;
	struct PRI_priv_data	*priv;
	int			ret;

	BUG_ON(!xpd);
	priv = xpd->priv;
	if(lc->span != xpd->span.spanno) {
		XPD_ERR(xpd, "I am span %d but got spanconfig for span %d\n",
			xpd->span.spanno, lc->span);
		return -EINVAL;
	}
	/*
	 * FIXME: lc->name is unused by ztcfg and zaptel...
	 *        We currently ignore it also.
	 */
	XPD_DBG(GENERAL, xpd, "[%s] lbo=%d lineconfig=0x%X sync=%d\n",
		(priv->is_nt)?"NT":"TE", lc->lbo, lc->lineconfig, lc->sync);
	ret = pri_lineconfig(xpd, lc->lineconfig);
	if(!ret) {
		span->lineconfig = lc->lineconfig;
		xpd->timing_priority = lc->sync;
		set_master_mode("spanconfig", xpd);
		elect_syncer("PRI-master_mode");
	}
	return ret;
}

/*
 * Set signalling type (if appropriate)
 * Called from zaptel with spinlock held on chan. Must not call back
 * zaptel functions.
 */
static int pri_chanconfig(struct zt_chan *chan, int sigtype)
{
	DBG(GENERAL, "channel %d (%s) -> %s\n", chan->channo, chan->name, sig2str(sigtype));
	// FIXME: sanity checks:
	// - should be supported (within the sigcap)
	// - should not replace fxs <->fxo ??? (covered by previous?)
	return 0;
}

static int PRI_card_init(xbus_t *xbus, xpd_t *xpd)
{
	struct PRI_priv_data	*priv;
	int			ret = 0;
	xproto_table_t		*proto_table;

	BUG_ON(!xpd);
	XPD_DBG(GENERAL, xpd, "\n");
	xpd->type = XPD_TYPE_PRI;
	proto_table = &PROTO_TABLE(PRI);
	priv = xpd->priv;
	xpd->xops = &proto_table->xops;
#ifdef	CONFIG_PROC_FS
	XPD_DBG(PROC, xpd, "Creating PRI_INFO file\n");
	priv->pri_info = create_proc_entry(PROC_PRI_INFO_FNAME, 0644, xpd->proc_xpd_dir);
	if(!priv->pri_info) {
		XPD_ERR(xpd, "Failed to create proc '%s'\n", PROC_PRI_INFO_FNAME);
		ret = -ENOENT;
		goto err;
	}
	priv->pri_info->owner = THIS_MODULE;
	priv->pri_info->write_proc = proc_pri_info_write;
	priv->pri_info->read_proc = proc_pri_info_read;
	priv->pri_info->data = xpd;
	XPD_DBG(PROC, xpd, "Creating registers file\n");
	priv->regfile = create_proc_entry(PROC_REGISTER_FNAME, 0644, xpd->proc_xpd_dir);
	if(!priv->regfile) {
		XPD_ERR(xpd, "Failed to create proc file '%s'\n", PROC_REGISTER_FNAME);
		ret = -ENOENT;
		goto err;
	}
	priv->regfile->owner = THIS_MODULE;
	priv->regfile->write_proc = proc_xpd_register_write;
	priv->regfile->read_proc = proc_xpd_register_read;
	priv->regfile->data = xpd;
#endif
	/* Assume E1, changes later from user space */
	ret = set_pri_proto(xpd, PRI_PROTO_E1);
	if(ret < 0)
		goto err;
	ret = run_initialize_registers(xpd);
	if(ret < 0)
		goto err;
	/*
	 * initialization script should have set correct
	 * operating modes.
	 */
	if(!valid_pri_modes(xpd)) {
		XPD_NOTICE(xpd, "PRI protocol not set\n");
		goto err;
	}
	/*
	 * Must set default now, so layer1 polling (Register REG_FRS0) would
	 * give reliable results.
	 */
	ret = pri_lineconfig(xpd, ZT_CONFIG_CCS | ZT_CONFIG_CRC4 | ZT_CONFIG_HDB3);
	if(ret) {
		XPD_NOTICE(xpd, "Failed setting PRI default line config\n");
		goto err;
	}
	XPD_DBG(GENERAL, xpd, "done\n");
	for(ret = 0; ret < NUM_LEDS; ret++) {
		DO_LED(xpd, ret, PRI_LED_ON);
		msleep(20);
		DO_LED(xpd, ret, PRI_LED_OFF);
	}
	priv->initialized = 1;
	return 0;
err:
	clean_proc(xbus, xpd);
	XPD_ERR(xpd, "Failed initializing registers (%d)\n", ret);
	return ret;
}

static int PRI_card_remove(xbus_t *xbus, xpd_t *xpd)
{
	struct PRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	XPD_DBG(GENERAL, xpd, "\n");
	clean_proc(xbus, xpd);
	return 0;
}

static int PRI_card_zaptel_preregistration(xpd_t *xpd, bool on)
{
	xbus_t			*xbus;
	struct PRI_priv_data	*priv;
	int			i;
	
	BUG_ON(!xpd);
	xbus = xpd->xbus;
	priv = xpd->priv;
	BUG_ON(!xbus);
	XPD_DBG(GENERAL, xpd, "%s (proto=%s, channels=%d, deflaw=%d)\n",
		(on)?"on":"off",
		pri_protocol_name(priv->pri_protocol),
		xpd->channels,
		priv->deflaw);
	if(!on) {
		/* Nothing to do yet */
		return 0;
	}
#ifdef ZT_SPANSTAT_V2 
	xpd->span.spantype = pri_protocol_name(priv->pri_protocol);
#endif 
	xpd->span.linecompat = pri_linecompat(priv->pri_protocol);
	xpd->span.deflaw = priv->deflaw;
	for_each_line(xpd, i) {
		struct zt_chan	*cur_chan = &xpd->chans[i];
		bool		is_dchan = i == PRI_DCHAN_IDX(priv);

		XPD_DBG(GENERAL, xpd, "setting PRI channel %d (%s)\n", i,
			(is_dchan)?"DCHAN":"CLEAR");
		snprintf(cur_chan->name, MAX_CHANNAME, "XPP_%s/%02d/%1d%1d/%d",
				xpd->type_name, xbus->num, xpd->addr.unit, xpd->addr.subunit, i);
		cur_chan->chanpos = i + 1;
		cur_chan->pvt = xpd;
		if(is_dchan) {	/* D-CHAN */
			cur_chan->sigcap = PRI_DCHAN_SIGCAP;
			//FIXME: cur_chan->flags |= ZT_FLAG_PRIDCHAN;
			cur_chan->flags &= ~ZT_FLAG_HDLC;
		} else
			cur_chan->sigcap = PRI_BCHAN_SIGCAP;
	}
	xpd->offhook = xpd->wanted_pcm_mask;
	xpd->span.spanconfig = pri_spanconfig;
	xpd->span.chanconfig = pri_chanconfig;
	xpd->span.startup = pri_startup;
	xpd->span.shutdown = pri_shutdown;
	return 0;
}

static int PRI_card_zaptel_postregistration(xpd_t *xpd, bool on)
{
	xbus_t			*xbus;
	struct PRI_priv_data	*priv;
	
	BUG_ON(!xpd);
	xbus = xpd->xbus;
	priv = xpd->priv;
	BUG_ON(!xbus);
	XPD_DBG(GENERAL, xpd, "%s\n", (on)?"on":"off");
	zap_update_syncsrc(xpd);
	return(0);
}

static int PRI_card_hooksig(xbus_t *xbus, xpd_t *xpd, int pos, zt_txsig_t txsig)
{
	LINE_DBG(SIGNAL, xpd, pos, "%s\n", txsig2str(txsig));
	return 0;
}

static void dchan_state(xpd_t *xpd, bool up)
{
	struct PRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	if(priv->dchan_alive == up)
		return;
	if(!priv->layer1_up)	/* No layer1, kill dchan */
		up = 0;
	if(up) {
		XPD_DBG(SIGNAL, xpd, "STATE CHANGE: D-Channel RUNNING\n");
		priv->dchan_alive = 1;
	} else {
		int	d = PRI_DCHAN_IDX(priv);

		if(SPAN_REGISTERED(xpd) && d >= 0 && d < xpd->channels) {
			byte	*pcm;

			pcm = (byte *)xpd->span.chans[d].readchunk;
			pcm[0] = 0x00;
			pcm = (byte *)xpd->span.chans[d].writechunk;
			pcm[0] = 0x00;
		}
		XPD_DBG(SIGNAL, xpd, "STATE CHANGE: D-Channel STOPPED\n");
		priv->dchan_rx_counter = priv->dchan_tx_counter = 0;
		priv->dchan_alive = 0;
		priv->dchan_alive_ticks = 0;
		priv->dchan_rx_sample = priv->dchan_tx_sample = 0x00;
	}
}

/*
 * LED managment is done by the driver now:
 *   - Turn constant ON RED/GREEN led to indicate NT/TE port
 *   - Very fast "Double Blink" to indicate Layer1 alive (without D-Channel)
 *   - Constant blink (1/2 sec cycle) to indicate D-Channel alive.
 */
static void handle_leds(xbus_t *xbus, xpd_t *xpd)
{
	struct PRI_priv_data	*priv;
	unsigned int		timer_count;
	int			which_led;
	int			other_led;
	enum pri_led_state	ledstate;
	int			mod;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	if(priv->is_nt) {
		which_led = NT_RED_LED;
		other_led = TE_GREEN_LED;
	} else {
		which_led = TE_GREEN_LED;
		other_led = NT_RED_LED;
	}
	ledstate = priv->ledstate[which_led];
	timer_count = xpd->timer_count;
	if(xpd->blink_mode) {
		if((timer_count % DEFAULT_LED_PERIOD) == 0) {
			// led state is toggled
			if(ledstate == PRI_LED_OFF) {
				DO_LED(xpd, which_led, PRI_LED_ON);
				DO_LED(xpd, other_led, PRI_LED_ON);
			} else {
				DO_LED(xpd, which_led, PRI_LED_OFF);
				DO_LED(xpd, other_led, PRI_LED_OFF);
			}
		}
		return;
	}
	if(priv->ledstate[other_led] != PRI_LED_OFF)
		DO_LED(xpd, other_led, PRI_LED_OFF);
	if(priv->dchan_alive) {
		mod = timer_count % 1000;
		switch(mod) {
			case 0:
				DO_LED(xpd, which_led, PRI_LED_ON);
				break;
			case 500:
				DO_LED(xpd, which_led, PRI_LED_OFF);
				break;
		}
	} else if(priv->layer1_up) {
		mod = timer_count % 1000;
		switch(mod) {
			case 0:
			case 100:
				DO_LED(xpd, which_led, PRI_LED_ON);
				break;
			case 50:
			case 150:
				DO_LED(xpd, which_led, PRI_LED_OFF);
				break;
		}
	} else {
		if(priv->ledstate[which_led] != PRI_LED_ON)
			DO_LED(xpd, which_led, PRI_LED_ON);
	}
}

static int PRI_card_tick(xbus_t *xbus, xpd_t *xpd)
{
	struct PRI_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	if(!priv->initialized || !xbus->self_ticking)
		return 0;
	/*
	 * Poll layer1 status (cascade subunits)
	 */
	if(poll_interval != 0 &&
		((xpd->timer_count % poll_interval) == 0)) {
		priv->poll_noreplies++;
		query_subunit(xpd, REG_FRS0);
		//query_subunit(xpd, REG_FRS1);
	}
	if(priv->dchan_tx_counter >= 1 && priv->dchan_rx_counter > 1) {
		dchan_state(xpd, 1);
		priv->dchan_alive_ticks++;
	}
	handle_leds(xbus, xpd);
	return 0;
}

static int PRI_card_ioctl(xpd_t *xpd, int pos, unsigned int cmd, unsigned long arg)
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
			LINE_DBG(SIGNAL, xpd, pos, "PRI: Starting a call\n");
			return -ENOTTY;
		default:
			report_bad_ioctl(THIS_MODULE->name, xpd, pos, cmd);
			return -ENOTTY;
	}
	return 0;
}

static int PRI_card_close(xpd_t *xpd, lineno_t pos)
{
	//struct zt_chan	*chan = &xpd->span.chans[pos];
	dchan_state(xpd, 0);
	return 0;
}

/*
 * Called only for 'span' keyword in /etc/zaptel.conf
 */
static int pri_startup(struct zt_span *span)
{
	xpd_t			*xpd = span->pvt;
	struct PRI_priv_data	*priv;

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
	return 0;
}

/*
 * Called only for 'span' keyword in /etc/zaptel.conf
 */
static int pri_shutdown(struct zt_span *span)
{
	xpd_t			*xpd = span->pvt;
	struct PRI_priv_data	*priv;

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
	return 0;
}

/*! Copy PCM chunks from the buffers of the xpd to a new packet
 * \param xbus	xbus of source xpd.
 * \param xpd	source xpd.
 * \param lines	a bitmask of the active channels that need to be copied. 
 * \param pack	packet to be filled.
 *
 * On PRI this function is should also shift the lines mask one bit, as
 * channel 0 on the wire is an internal chip control channel. We only
 * send 31 channels to the device, but they should be called 1-31 rather
 * than 0-30 .
 */
static void PRI_card_pcm_fromspan(xbus_t *xbus, xpd_t *xpd, xpp_line_t lines, xpacket_t *pack)
{
	struct PRI_priv_data	*priv;
	byte			*pcm;
	struct zt_chan		*chans;
	unsigned long		flags;
	int			i;
	int			physical_chan;
	int			physical_mask = 0;

	BUG_ON(!xbus);
	BUG_ON(!xpd);
	BUG_ON(!pack);
	priv = xpd->priv;
	BUG_ON(!priv);
	pcm = RPACKET_FIELD(pack, GLOBAL, PCM_WRITE, pcm);
	spin_lock_irqsave(&xpd->lock, flags);
	chans = xpd->span.chans;
	physical_chan = 0;
	for_each_line(xpd, i) {
		if(priv->pri_protocol == PRI_PROTO_E1) {
			/* In E1 - Only 0'th channel is unused */
			if(i == 0) {
				physical_chan++;
			}
		} else if(priv->pri_protocol == PRI_PROTO_T1) {
			/* In T1 - Every 4'th channel is unused */
			if((i % 3) == 0) {
				physical_chan++;
			}
		}
		if(IS_SET(lines, i)) {
			physical_mask |= BIT(physical_chan);
			if(SPAN_REGISTERED(xpd)) {
				if(i == PRI_DCHAN_IDX(priv)) {
					if(priv->dchan_tx_sample != chans[i].writechunk[0]) {
						priv->dchan_tx_sample = chans[i].writechunk[0];
						priv->dchan_tx_counter++;
					} else if(chans[i].writechunk[0] == 0xFF)
						dchan_state(xpd, 0);
				}
#ifdef	DEBUG_PCMTX
				if(pcmtx >= 0 && pcmtx_chan == i)
					memset((u_char *)pcm, pcmtx, ZT_CHUNKSIZE);
				else
#endif
					memcpy((u_char *)pcm, chans[i].writechunk, ZT_CHUNKSIZE);
				// fill_beep((u_char *)pcm, xpd->addr.subunit, 2);
			} else
				memset((u_char *)pcm, 0x7F, ZT_CHUNKSIZE);
			pcm += ZT_CHUNKSIZE;
		}
		physical_chan++;
	}
	RPACKET_FIELD(pack, GLOBAL, PCM_WRITE, lines) = physical_mask;
	XPD_COUNTER(xpd, PCM_WRITE)++;
	spin_unlock_irqrestore(&xpd->lock, flags);
}

/*! Copy PCM chunks from the packet we recieved to the xpd struct.
 * \param xbus	xbus of target xpd.
 * \param xpd	target xpd.
 * \param pack	Source packet.
 *
 * On PRI this function is should also shift the lines back mask one bit, as
 * channel 0 on the wire is an internal chip control channel. 
 *
 * \see PRI_card_pcm_fromspan
 */
static void PRI_card_pcm_tospan(xbus_t *xbus, xpd_t *xpd, xpacket_t *pack)
{
	struct PRI_priv_data	*priv;
	byte			*pcm;
	struct zt_chan		*chans;
	xpp_line_t		physical_mask;
	unsigned long		flags;
	int			i;
	int			logical_chan;

	if(!SPAN_REGISTERED(xpd))
		return;
	priv = xpd->priv;
	BUG_ON(!priv);
	pcm = RPACKET_FIELD(pack, GLOBAL, PCM_READ, pcm);
	physical_mask = RPACKET_FIELD(pack, GLOBAL, PCM_WRITE, lines);
	spin_lock_irqsave(&xpd->lock, flags);
	chans = xpd->span.chans;
	logical_chan = 0;
	for (i = 0; i < CHANNELS_PERXPD; i++) {
		volatile u_char	*r;

		if(priv->pri_protocol == PRI_PROTO_E1) {
			/* In E1 - Only 0'th channel is unused */
			if(i == 0)
				continue;
		} else if(priv->pri_protocol == PRI_PROTO_T1) {
			/* In T1 - Every 4'th channel is unused */
			if((i % 4) == 0)
				continue;
		}
		if(logical_chan == PRI_DCHAN_IDX(priv)) {
			if(priv->dchan_rx_sample != pcm[0]) {
				if(print_dbg & DBG_PCM) {
					XPD_INFO(xpd, "RX-D-Chan: prev=0x%X now=0x%X\n",
							priv->dchan_rx_sample, pcm[0]);
					dump_packet("RX-D-Chan", pack, 1);
				}
				priv->dchan_rx_sample = pcm[0];
				priv->dchan_rx_counter++;
			} else if(pcm[0] == 0xFF)
				dchan_state(xpd, 0);
		}
		if(IS_SET(physical_mask, i)) {
			r = chans[logical_chan].readchunk;
			// memset((u_char *)r, 0x5A, ZT_CHUNKSIZE);	// DEBUG
			// fill_beep((u_char *)r, 1, 1);	// DEBUG: BEEP
			memcpy((u_char *)r, pcm, ZT_CHUNKSIZE);
			pcm += ZT_CHUNKSIZE;
		}
		logical_chan++;
	}
	XPD_COUNTER(xpd, PCM_READ)++;
	spin_unlock_irqrestore(&xpd->lock, flags);
}

/*---------------- PRI: HOST COMMANDS -------------------------------------*/

static /* 0x0F */ HOSTCMD(PRI, XPD_STATE, bool on)
{
	BUG_ON(!xpd);
	XPD_DBG(GENERAL, xpd, "%s\n", (on)?"on":"off");
	return 0;
}

static /* 0x0F */ HOSTCMD(PRI, RING, lineno_t chan, bool on)
{
	XPD_ERR(xpd, "%s: Unsupported\n", __FUNCTION__);
	return -ENOSYS;
}

static /* 0x0F */ HOSTCMD(PRI, RELAY_OUT, byte which, bool on)
{
	XPD_ERR(xpd, "%s: Unsupported\n", __FUNCTION__);
	return -ENOSYS;
}

/* 0x33 */ HOSTCMD(PRI, SET_LED, enum pri_led_selectors led_sel, enum pri_led_state to_led_state)
{
	int			ret = 0;
	xframe_t		*xframe;
	xpacket_t		*pack;
	struct pri_leds		*pri_leds;
	struct PRI_priv_data	*priv;

	BUG_ON(!xbus);
	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	XPD_DBG(LEDS, xpd, "led_sel=%d to_state=%d\n", led_sel, to_led_state);
	XFRAME_NEW_CMD(xframe, pack, xbus, PRI, SET_LED, xpd->xbus_idx);
	pri_leds = &RPACKET_FIELD(pack, PRI, SET_LED, pri_leds);
	pri_leds->state = to_led_state;
	pri_leds->led_sel = led_sel;
	XPACKET_LEN(pack) = RPACKET_SIZE(PRI, SET_LED);
	ret = send_cmd_frame(xbus, xframe);
	priv->ledstate[led_sel] = to_led_state;
	return ret;
}

/*---------------- PRI: Astribank Reply Handlers --------------------------*/
static void layer1_state(xpd_t *xpd, byte subunit, byte data_low)
{
	struct PRI_priv_data	*priv;
	int			alarms = 0;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	if(xpd->addr.subunit != subunit) {
		XPD_NOTICE(xpd, "layer1_state got wrong subunit=%d. Ignored.\n", subunit);
		return;
	}
	priv->poll_noreplies = 0;
	if(data_low & REG_FRS0_LOS)
		alarms |=  ZT_ALARM_RED;
	if(data_low & REG_FRS0_AIS)
		alarms |= ZT_ALARM_BLUE;
	if(data_low & REG_FRS0_RRA)
		alarms |= ZT_ALARM_YELLOW;
	priv->layer1_up = alarms == 0;
	/*
	 * Some bad bits (e.g: LMFA and NMF have no alarm "colors"
	 * associated. However, layer1 is still not working if they are set.
	 */
	if(data_low & (REG_FRS0_LMFA | REG_FRS0_NMF))
		priv->layer1_up = 0;
	priv->alarms = alarms;
	if(!priv->layer1_up)
		dchan_state(xpd, 0);
	if(SPAN_REGISTERED(xpd) && xpd->span.alarms != alarms) {
		char	str1[MAX_PROC_WRITE];
		char	str2[MAX_PROC_WRITE];

		alarm2str(xpd->span.alarms, str1, sizeof(str1));
		alarm2str(alarms, str2, sizeof(str2));
		XPD_NOTICE(xpd, "Alarms: 0x%X (%s) => 0x%X (%s)\n",
				xpd->span.alarms, str1,
				alarms, str2);
		xpd->span.alarms = alarms;
		zt_alarm_notify(&xpd->span);
		set_clocking(xpd);
	}
	priv->reg_frs0 = data_low;
	priv->layer1_replies++;
	XPD_DBG(REGS, xpd, "subunit=%d data_low=0x%02X\n", subunit, data_low);
}

static int PRI_card_register_reply(xbus_t *xbus, xpd_t *xpd, reg_cmd_t *info)
{
	unsigned long		flags;
	struct PRI_priv_data	*priv;

	spin_lock_irqsave(&xpd->lock, flags);
	priv = xpd->priv;
	BUG_ON(!priv);
#if 1
	if(print_dbg)
		dump_reg_cmd("PRI", info, 0);
#endif
	if(info->multibyte) {
		XPD_NOTICE(xpd, "Got Multibyte: %d bytes, eoframe: %d\n",
				info->bytes, info->eoframe);
		goto end;
	}
	if(REG_FIELD(info, regnum) == REG_FRS0 && REG_FIELD(info, do_subreg))
		layer1_state(xpd, REG_FIELD(info, subreg), REG_FIELD(info, data_low));
	if(REG_FIELD(info, regnum) == REG_FRS1 && REG_FIELD(info, do_subreg))
		priv->reg_frs1 = REG_FIELD(info, data_low);
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

static xproto_table_t PROTO_TABLE(PRI) = {
	.owner = THIS_MODULE,
	.entries = {
		/*	Table	Card	Opcode		*/
	},
	.name = "PRI_xx",	/* xpd->type_name is set in set_nt() */
	.type = XPD_TYPE_PRI,
	.xops = {
		.card_new	= PRI_card_new,
		.card_init	= PRI_card_init,
		.card_remove	= PRI_card_remove,
		.card_zaptel_preregistration	= PRI_card_zaptel_preregistration,
		.card_zaptel_postregistration	= PRI_card_zaptel_postregistration,
		.card_hooksig	= PRI_card_hooksig,
		.card_tick	= PRI_card_tick,
		.card_pcm_fromspan	= PRI_card_pcm_fromspan,
		.card_pcm_tospan	= PRI_card_pcm_tospan,
		.card_ioctl	= PRI_card_ioctl,
		.card_close	= PRI_card_close,
		.card_register_reply	= PRI_card_register_reply,

		.RING		= XPROTO_CALLER(PRI, RING),
		.RELAY_OUT	= XPROTO_CALLER(PRI, RELAY_OUT),
		.XPD_STATE	= XPROTO_CALLER(PRI, XPD_STATE),
	},
	.packet_is_valid = pri_packet_is_valid,
	.packet_dump = pri_packet_dump,
};

static bool pri_packet_is_valid(xpacket_t *pack)
{
	const xproto_entry_t	*xe_nt = NULL;
	const xproto_entry_t	*xe_te = NULL;
	// DBG(GENERAL, "\n");
	xe_nt = xproto_card_entry(&PROTO_TABLE(PRI), XPACKET_OP(pack));
	return xe_nt != NULL || xe_te != NULL;
}

static void pri_packet_dump(const char *msg, xpacket_t *pack)
{
	DBG(GENERAL, "%s\n", msg);
}
/*------------------------- REGISTER Handling --------------------------*/
static int proc_pri_info_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	xpd_t			*xpd = data;
	struct PRI_priv_data	*priv;
	char			buf[MAX_PROC_WRITE];
	char			*p;
	char			*tok;
	static const char	*msg = "PROC";	/* for logs */
	int			ret = 0;
	bool			got_localloop = 0;
	bool			got_nolocalloop = 0;
	bool			got_te = 0;
	bool			got_nt = 0;
	bool			got_e1 = 0;
	bool			got_t1 = 0;
	bool			got_j1 = 0;

	if(!xpd)
		return -ENODEV;
	priv = xpd->priv;
	if(count >= MAX_PROC_WRITE) {	/* leave room for null */
		XPD_ERR(xpd, "write too long (%ld)\n", count);
		return -E2BIG;
	}
	if(copy_from_user(buf, buffer, count)) {
		XPD_ERR(xpd, "Failed reading user data\n");
		return -EFAULT;
	}
	buf[count] = '\0';
	XPD_DBG(PROC, xpd, "PRI-SETUP: got %s\n", buf);
	/*
	 * First parse. Act only of *everything* is good.
	 */
	p = buf;
	while((tok = strsep(&p, " \t\v\n")) != NULL) {
		if(*tok == '\0')
			continue;
		XPD_DBG(PROC, xpd, "Got token='%s'\n", tok);
		if(strnicmp(tok, "LOCALLOOP", 8) == 0)
			got_localloop = 1;
		else if(strnicmp(tok, "NOLOCALLOOP", 8) == 0)
			got_nolocalloop = 1;
		else if(strnicmp(tok, "NT", 2) == 0)
			got_nt = 1;
		else if(strnicmp(tok, "TE", 2) == 0)
			got_te = 1;
		else if(strnicmp(tok, "E1", 2) == 0)
			got_e1 = 1;
		else if(strnicmp(tok, "T1", 2) == 0)
			got_t1 = 1;
		else if(strnicmp(tok, "J1", 2) == 0) {
			got_j1 = 1;
		} else {
			XPD_NOTICE(xpd, "PRI-SETUP: unknown keyword: '%s'\n", tok);
			return -EINVAL;
		}
	}
	if(got_e1)
		ret = set_pri_proto(xpd, PRI_PROTO_E1);
	else if(got_t1)
		ret = set_pri_proto(xpd, PRI_PROTO_T1);
	else if(got_j1)
		ret = set_pri_proto(xpd, PRI_PROTO_J1);
	if(priv->pri_protocol == PRI_PROTO_0) {
		XPD_ERR(xpd,
			"Must set PRI protocol (E1/T1/J1) before setting other parameters\n");
		return -EINVAL;
	}
	if(got_localloop)
		ret = set_localloop(msg, xpd, 1);
	if(got_nolocalloop)
		ret = set_localloop(msg, xpd, 0);
	if(got_nt)
		ret = set_nt(msg, xpd, 1);
	if(got_te)
		ret = set_nt(msg, xpd, 0);
	return (ret) ? ret : count;
}


static int proc_pri_info_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int			len = 0;
	unsigned long		flags;
	xpd_t			*xpd = data;
	struct PRI_priv_data	*priv;
	int			i;

	DBG(PROC, "\n");
	if(!xpd)
		return -ENODEV;
	spin_lock_irqsave(&xpd->lock, flags);
	priv = xpd->priv;
	BUG_ON(!priv);
	len += sprintf(page + len, "PRI: %s %s%s (deflaw=%d, dchan=%d)\n",
		(priv->is_nt) ? "NT" : "TE",
		pri_protocol_name(priv->pri_protocol),
		(priv->local_loopback) ? " LOCALLOOP" : "",
		priv->deflaw, priv->dchan_num);
	len += sprintf(page + len, "%05d Layer1: ", priv->layer1_replies);
	if(priv->poll_noreplies > 1)
		len += sprintf(page + len, "No Replies [%d]\n",
			priv->poll_noreplies);
	else {
		len += sprintf(page + len, "%s\n",
				((priv->layer1_up) ?  "UP" : "DOWN"));
		len += sprintf(page + len,
				"Framer Status: FRS0=0x%02X, FRS1=0x%02X ALARMS:",
				priv->reg_frs0, priv->reg_frs1);
		if(priv->reg_frs0 & REG_FRS0_LOS)
			len += sprintf(page + len, " RED");
		if(priv->reg_frs0 & REG_FRS0_AIS)
			len += sprintf(page + len, " BLUE");
		if(priv->reg_frs0 & REG_FRS0_RRA)
			len += sprintf(page + len, " YELLOW");
		len += sprintf(page + len, "\n");
	}
	len += sprintf(page + len, "D-Channel: TX=[%5d] (0x%02X)   RX=[%5d] (0x%02X) ",
			priv->dchan_tx_counter, priv->dchan_tx_sample,
			priv->dchan_rx_counter, priv->dchan_rx_sample);
	if(priv->dchan_alive) {
		len += sprintf(page + len, "(alive %d K-ticks)\n",
			priv->dchan_alive_ticks/1000);
	} else {
		len += sprintf(page + len, "(dead)\n");
	}
	for(i = 0; i < NUM_LEDS; i++)
		len += sprintf(page + len, "LED #%d: %d\n", i, priv->ledstate[i]);
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
	struct PRI_priv_data	*priv;
	byte			buf[MAX_PROC_WRITE];

	BUG_ON(!xpd);
	xbus = xpd->xbus;
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
			(op == 'R' && reg_type == 'S' && elements != 5)
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
			XPD_DBG(PROC, xpd, "SUBREG\n");
			break;
		case 'D':
			REG_FIELD(&regcmd, do_subreg) = 0;
			REG_FIELD(&regcmd, regnum) = reg_num;
			REG_FIELD(&regcmd, subreg) = 0;
			REG_FIELD(&regcmd, data_low) = subreg;
			XPD_DBG(PROC, xpd, "DIRECT\n");
			break;
		default:
			ERR("Unkown register type '%c'\n", reg_type);
			goto out;
	}
	regcmd.bytes = sizeof(regcmd) - 1;
	REG_FIELD(&regcmd, read_request) = writing;
	REG_FIELD(&regcmd, data_high) = 0;
	priv->requested_reply = regcmd;
	if(print_dbg)
		dump_reg_cmd("PRI", &regcmd, 1);
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
	struct PRI_priv_data	*priv;

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

int __init card_pri_startup(void)
{
	DBG(GENERAL, "\n");

	INFO("revision %s\n", XPP_VERSION);
	xproto_register(&PROTO_TABLE(PRI));
	return 0;
}

void __exit card_pri_cleanup(void)
{
	DBG(GENERAL, "\n");
	xproto_unregister(&PROTO_TABLE(PRI));
}

MODULE_DESCRIPTION("XPP PRI Card Driver");
MODULE_AUTHOR("Oron Peled <oron@actcom.co.il>");
MODULE_LICENSE("GPL");
MODULE_VERSION(XPP_VERSION);
MODULE_ALIAS_XPD(XPD_TYPE_PRI);

module_init(card_pri_startup);
module_exit(card_pri_cleanup);
