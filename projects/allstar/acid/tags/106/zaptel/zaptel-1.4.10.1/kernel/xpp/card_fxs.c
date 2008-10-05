/*
 * Written by Oron Peled <oron@actcom.co.il>
 * Copyright (C) 2004-2006, Xorcom
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
#include "card_fxo.h"
#include "zap_debug.h"
#include "xbus-core.h"

static const char rcsid[] = "$Id: card_fxs.c 4013 2008-03-19 01:19:01Z tzafrir $";

DEF_PARM(int, print_dbg, 0, 0644, "Print DBG statements");	/* must be before zap_debug.h */
DEF_PARM_BOOL(reversepolarity, 0, 0644, "Reverse Line Polarity");
DEF_PARM_BOOL(vmwineon, 0, 0644, "Indicate voicemail to a neon lamp");
DEF_PARM_BOOL(dtmf_detection, 1, 0644, "Do DTMF detection in hardware");
#ifdef	POLL_DIGITAL_INPUTS
DEF_PARM(uint, poll_digital_inputs, 1000, 0644, "Poll Digital Inputs");
#endif

#ifdef	ZT_VMWI
DEF_PARM_BOOL(vmwi_ioctl, 0, 0644, "Asterisk support VMWI notification via ioctl");
#else
#define	vmwi_ioctl	0	/* not supported */
#endif

/* Signaling is opposite (fxo signalling for fxs card) */
#if 1
#define	FXS_DEFAULT_SIGCAP	(ZT_SIG_FXOKS | ZT_SIG_FXOLS | ZT_SIG_FXOGS)
#else
#define	FXS_DEFAULT_SIGCAP	(ZT_SIG_SF | ZT_SIG_EM)
#endif

#define	LINES_DIGI_OUT	2
#define	LINES_DIGI_INP	4

enum fxs_leds {
	LED_GREEN,
	LED_RED,
	OUTPUT_RELAY,
};

#define	NUM_LEDS	2

/* Shortcuts */
#define	SLIC_WRITE	1
#define	SLIC_READ	0
#define	SLIC_DIRECT_REQUEST(xbus,xpd,chipsel,writing,reg,dL)	\
	xpp_register_request((xbus), (xpd), (chipsel), (writing), 0, (reg), 0, (dL), 0)
#define	SLIC_INDIRECT_REQUEST(xbus,xpd,chipsel,writing,reg,dL,dH)	\
	xpp_register_request((xbus), (xpd), (chipsel), (writing), 1, 0x1E, (reg), (dL), (dH))

#define	VALID_CHIPSEL(x)	(((chipsel) >= 0 && (chipsel) <= 7) || (chipsel) == ALL_CHANS)

/* Values of SLIC linefeed control register (0x40) */
enum fxs_state {
	FXS_LINE_OPEN		= 0x00,	/* Open */
	FXS_LINE_ACTIVE		= 0x01,	/* Forward active */
	FXS_LINE_OHTRANS	= 0x02,	/* Forward on-hook transmission */
	FXS_LINE_TIPOPEN	= 0x03,	/* TIP open */
	FXS_LINE_RING		= 0x04,	/* Ringing */
	FXS_LINE_REV_ACTIVE	= 0x05,	/* Reverse active */
	FXS_LINE_REV_OHTRANS	= 0x06,	/* Reverse on-hook transmission */
	FXS_LINE_RING_OPEN	= 0x07	/* RING open */
};

#define	FXS_LINE_POL_ACTIVE	((reversepolarity) ? FXS_LINE_REV_ACTIVE : FXS_LINE_ACTIVE)
#define	FXS_LINE_POL_OHTRANS	((reversepolarity) ? FXS_LINE_REV_OHTRANS : FXS_LINE_OHTRANS)

/*
 * DTMF detection
 */
#define SLIC_REG_DTMF		0x18	/* 24 */
#define SLIC_REG_VOLTAGE	0x42	/* 66 */

/*---------------- FXS Protocol Commands ----------------------------------*/

static /* 0x0F */ DECLARE_CMD(FXS, XPD_STATE, bool on);
static /* 0x0F */ DECLARE_CMD(FXS, RING, lineno_t chan, bool on);
static /* 0x0F */ DECLARE_CMD(FXS, RELAY_OUT, byte which, bool on);

static bool fxs_packet_is_valid(xpacket_t *pack);
static void fxs_packet_dump(const char *msg, xpacket_t *pack);
static int proc_fxs_info_read(char *page, char **start, off_t off, int count, int *eof, void *data);
#ifdef	WITH_METERING
static int proc_xpd_metering_write(struct file *file, const char __user *buffer, unsigned long count, void *data);
#endif
static int proc_xpd_register_read(char *page, char **start, off_t off, int count, int *eof, void *data);
static int proc_xpd_register_write(struct file *file, const char __user *buffer, unsigned long count, void *data);
static void start_stop_vm_led(xbus_t *xbus, xpd_t *xpd, lineno_t pos);

#define	PROC_REGISTER_FNAME	"slics"
#define	PROC_FXS_INFO_FNAME	"fxs_info"
#ifdef	WITH_METERING
#define	PROC_METERING_FNAME	"metering_gen"
#endif

struct FXS_priv_data {
	struct proc_dir_entry	*regfile;
#ifdef	WITH_METERING
	struct proc_dir_entry	*meteringfile;
#endif
	struct proc_dir_entry	*fxs_info;
	xpp_line_t		ledstate[NUM_LEDS];	/* 0 - OFF, 1 - ON */
	xpp_line_t		ledcontrol[NUM_LEDS];	/* 0 - OFF, 1 - ON */
	xpp_line_t		search_fsk_pattern;
	xpp_line_t		found_fsk_pattern;
	xpp_line_t		update_offhook_state;
	xpp_line_t		want_dtmf_events;	/* what zaptel want */
	xpp_line_t		want_dtmf_mute;		/* what zaptel want */
	int			led_counter[NUM_LEDS][CHANNELS_PERXPD];
	int			ohttimer[CHANNELS_PERXPD];
#define OHT_TIMER		6000	/* How long after RING to retain OHT */
	enum fxs_state		idletxhookstate[CHANNELS_PERXPD];	/* IDLE changing hook state */
	enum fxs_state		lasttxhook[CHANNELS_PERXPD];
};

/*
 * LED counter values:
 *	n>1	: BLINK every n'th tick
 */
#define	LED_COUNTER(priv,pos,color)	((priv)->led_counter[color][pos])
#define	IS_BLINKING(priv,pos,color)	(LED_COUNTER(priv,pos,color) > 0)
#define	MARK_BLINK(priv,pos,color,t)	((priv)->led_counter[color][pos] = (t))
#define	MARK_OFF(priv,pos,color)	do { BIT_CLR((priv)->ledcontrol[color],(pos)); MARK_BLINK((priv),(pos),(color),0); } while(0)
#define	MARK_ON(priv,pos,color)		do { BIT_SET((priv)->ledcontrol[color],(pos)); MARK_BLINK((priv),(pos),(color),0); } while(0)

#define	LED_BLINK_RING			(1000/8)	/* in ticks */

/*---------------- FXS: Static functions ----------------------------------*/
static int linefeed_control(xbus_t *xbus, xpd_t *xpd, lineno_t chan, enum fxs_state value)
{
	struct FXS_priv_data	*priv;

	priv = xpd->priv;
	LINE_DBG(SIGNAL, xpd, chan, "value=0x%02X\n", value);
	priv->lasttxhook[chan] = value;
	return SLIC_DIRECT_REQUEST(xbus, xpd, chan, SLIC_WRITE, 0x40, value);
}

static int do_chan_power(xbus_t *xbus, xpd_t *xpd, lineno_t chan, bool on)
{
	int		value = (on) ? 0x06 : 0x00;

	BUG_ON(!xbus);
	BUG_ON(!xpd);
	LINE_DBG(SIGNAL, xpd, chan, "%s\n", (on) ? "up" : "down");
	return SLIC_DIRECT_REQUEST(xbus, xpd, chan, SLIC_WRITE, SLIC_REG_VOLTAGE, value);
}

/*
 * LED and RELAY control is done via SLIC register 0x06:
 *         7     6     5     4     3     2     1     0
 * 	+-----+-----+-----+-----+-----+-----+-----+-----+
 * 	| M2  | M1  | M3  | C2  | O1  | O3  | C1  | C3  |
 * 	+-----+-----+-----+-----+-----+-----+-----+-----+
 *
 * 	Cn	- Control bit (control one digital line)
 * 	On	- Output bit (program a digital line for output)
 * 	Mn	- Mask bit (only the matching output control bit is affected)
 *
 * 	C3	- OUTPUT RELAY (0 - OFF, 1 - ON)
 * 	C1	- GREEN LED (0 - OFF, 1 - ON)
 * 	O3	- Output RELAY (this line is output)
 * 	O1	- Output GREEN (this line is output)
 * 	C2	- RED LED (0 - OFF, 1 - ON)
 * 	M3	- Mask RELAY. (1 - C3 effect the OUTPUT RELAY)
 * 	M2	- Mask RED. (1 - C2 effect the RED LED)
 * 	M1	- Mask GREEN. (1 - C1 effect the GREEN LED)
 *
 * 	The OUTPUT RELAY (actually a relay out) is connected to line 0 and 4 only.
 */

//		        		       		GREEN	RED	OUTPUT RELAY
static const int	led_register_mask[] = { 	BIT(7),	BIT(6),	BIT(5) };
static const int	led_register_vals[] = { 	BIT(4),	BIT(1),	BIT(0) };

/*
 * pos can be:
 * 	- A line number
 * 	- ALL_LINES. This is not valid anymore since 8-Jan-2007.
 */
static int do_led(xpd_t *xpd, lineno_t chan, byte which, bool on)
{
	int			ret = 0;
	struct FXS_priv_data	*priv;
	int			value;
	xbus_t			*xbus;

	BUG_ON(!xpd);
	BUG_ON(chan == ALL_LINES);
	xbus = xpd->xbus;
	priv = xpd->priv;
	which = which % NUM_LEDS;
	if(IS_SET(xpd->digital_outputs, chan) || IS_SET(xpd->digital_inputs, chan))
		goto out;
	if(chan == ALL_CHANS) {
		priv->ledstate[which] = (on) ? ~0 : 0;
	} else {
		if(on) {
			BIT_SET(priv->ledstate[which], chan);
		} else {
			BIT_CLR(priv->ledstate[which], chan);
		}
	}
	LINE_DBG(LEDS, xpd, chan, "LED: which=%d -- %s\n", which, (on) ? "on" : "off");
	value = BIT(2) | BIT(3);
	value |= ((BIT(5) | BIT(6) | BIT(7)) & ~led_register_mask[which]);
	if(on)
		value |= led_register_vals[which];
	ret = SLIC_DIRECT_REQUEST(xbus, xpd, chan, SLIC_WRITE, 0x06, value);
out:
	return ret;
}

static void handle_fxs_leds(xpd_t *xpd)
{
	int			i;
	const enum fxs_leds	colors[] = { LED_GREEN, LED_RED };
	int			color;
	unsigned int		timer_count;
	struct FXS_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	timer_count = xpd->timer_count;
	for(color = 0; color < ARRAY_SIZE(colors); color++) {
		for_each_line(xpd, i) {
			if(IS_SET(xpd->digital_outputs | xpd->digital_inputs, i))
				continue;
			if(xpd->blink_mode || IS_BLINKING(priv, i, color)) {		// Blinking
				int	mod_value = LED_COUNTER(priv, i, color);

				if(!mod_value)
					mod_value = DEFAULT_LED_PERIOD;		/* safety value */
				// led state is toggled
				if((timer_count % mod_value) == 0) {
					LINE_DBG(LEDS, xpd, i, "ledstate=%s\n", (IS_SET(priv->ledstate[color], i))?"ON":"OFF");
					if(!IS_SET(priv->ledstate[color], i)) {
						do_led(xpd, i, color, 1);
					} else {
						do_led(xpd, i, color, 0);
					}
				}
			} else if(IS_SET(priv->ledcontrol[color] & ~priv->ledstate[color], i)) {
						do_led(xpd, i, color, 1);
			} else if(IS_SET(~priv->ledcontrol[color] & priv->ledstate[color], i)) {
						do_led(xpd, i, color, 0);
			}

		}
	}
}

static void restore_leds(xpd_t *xpd)
{
	struct FXS_priv_data	*priv;
	int			i;

	priv = xpd->priv;
	for_each_line(xpd, i) {
		if(IS_SET(xpd->offhook, i))
			MARK_ON(priv, i, LED_GREEN);
		else
			MARK_OFF(priv, i, LED_GREEN);
	}
}

#ifdef	WITH_METERING
static int metering_gen(xpd_t *xpd, lineno_t chan, bool on)
{
	byte	value = (on) ? 0x94 : 0x00;

	LINE_DBG(SIGNAL, xpd, chan, "METERING Generate: %s\n", (on)?"ON":"OFF");
	return SLIC_DIRECT_REQUEST(xpd->xbus, xpd, chan, SLIC_WRITE, 0x23, value);
}
#endif

/*---------------- FXS: Methods -------------------------------------------*/

static xpd_t *FXS_card_new(xbus_t *xbus, int unit, int subunit, const xproto_table_t *proto_table, byte subtype, byte revision)
{
	xpd_t		*xpd = NULL;
	int		channels;
	int		regular_channels;

	if(subtype == 2)
		regular_channels = min(6, CHANNELS_PERXPD);
	else
		regular_channels = min(8, CHANNELS_PERXPD);
	channels = regular_channels;
	if(unit == 0)
		channels += 6;	/* 2 DIGITAL OUTPUTS, 4 DIGITAL INPUTS */
	xpd = xpd_alloc(sizeof(struct FXS_priv_data), proto_table, channels);
	if(!xpd)
		return NULL;
	if(unit == 0) {
		XBUS_DBG(GENERAL, xbus, "First XPD detected. Initialize digital outputs/inputs\n");
		xpd->digital_outputs = BITMASK(LINES_DIGI_OUT) << regular_channels;
		xpd->digital_inputs = BITMASK(LINES_DIGI_INP) << (regular_channels + LINES_DIGI_OUT);
	}
	xpd->direction = TO_PHONE;
	xpd->revision = revision;
	xpd->type_name = proto_table->name;
	return xpd;
}

static void clean_proc(xbus_t *xbus, xpd_t *xpd)
{
	struct FXS_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
#ifdef	CONFIG_PROC_FS
	if(priv->regfile) {
		XPD_DBG(PROC, xpd, "Removing xpd SLIC file\n");
		priv->regfile->data = NULL;
		remove_proc_entry(PROC_REGISTER_FNAME, xpd->proc_xpd_dir);
		priv->regfile = NULL;
	}
#ifdef	WITH_METERING
	if(priv->meteringfile) {
		XPD_DBG(PROC, xpd, "Removing xpd metering tone file\n");
		priv->meteringfile->data = NULL;
		remove_proc_entry(PROC_METERING_FNAME, xpd->proc_xpd_dir);
		priv->meteringfile = NULL;
	}
#endif
	if(priv->fxs_info) {
		XPD_DBG(PROC, xpd, "Removing xpd FXS_INFO file\n");
		remove_proc_entry(PROC_FXS_INFO_FNAME, xpd->proc_xpd_dir);
		priv->fxs_info = NULL;
	}
#endif
}

static int FXS_card_init(xbus_t *xbus, xpd_t *xpd)
{
	struct FXS_priv_data	*priv;
	int			ret = 0;
	int			i;

	BUG_ON(!xpd);
	priv = xpd->priv;
#ifdef	CONFIG_PROC_FS
	XPD_DBG(PROC, xpd, "Creating FXS_INFO file\n");
	priv->fxs_info = create_proc_read_entry(PROC_FXS_INFO_FNAME, 0444, xpd->proc_xpd_dir, proc_fxs_info_read, xpd);
	if(!priv->fxs_info) {
		XPD_ERR(xpd, "Failed to create proc file '%s'\n", PROC_FXS_INFO_FNAME);
		ret = -ENOENT;
		goto err;
	}
	priv->fxs_info->owner = THIS_MODULE;
#ifdef	WITH_METERING
	XPD_DBG(PROC, xpd, "Creating Metering tone file\n");
	priv->meteringfile = create_proc_entry(PROC_METERING_FNAME, 0200, xpd->proc_xpd_dir);
	if(!priv->meteringfile) {
		XPD_ERR(xpd, "Failed to create proc file '%s'\n", PROC_METERING_FNAME);
		ret = -ENOENT;
		goto err;
	}
	priv->meteringfile->owner = THIS_MODULE;
	priv->meteringfile->write_proc = proc_xpd_metering_write;
	priv->meteringfile->read_proc = NULL;
	priv->meteringfile->data = xpd;
#endif
	XPD_DBG(PROC, xpd, "Creating SLICs file\n");
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
	for_each_line(xpd, i) {
		priv->idletxhookstate[i] = FXS_LINE_POL_ACTIVE;
	}
	ret = run_initialize_registers(xpd);
	if(ret < 0)
		goto err;
	/*
	 * Setup ring timers
	 */
	/* Software controled ringing (for CID) */
	ret = SLIC_DIRECT_REQUEST(xbus, xpd, ALL_CHANS, SLIC_WRITE, 0x22, 0x00);	/* Ringing Oscilator Control */
	if(ret < 0)
		goto err;
	XPD_DBG(GENERAL, xpd, "done\n");
	for_each_line(xpd, i) {
		do_led(xpd, i, LED_GREEN, 0);
		do_led(xpd, i, LED_RED, 0);
	}
	for_each_line(xpd, i) {
		do_led(xpd, i, LED_GREEN, 1);
		msleep(50);
	}
	for_each_line(xpd, i) {
		do_led(xpd, i, LED_GREEN, 0);
		msleep(50);
	}
	restore_leds(xpd);
	pcm_recompute(xpd, 0);
	return 0;
err:
	clean_proc(xbus, xpd);
	XPD_ERR(xpd, "Failed initializing registers (%d)\n", ret);
	return ret;
}

static int FXS_card_remove(xbus_t *xbus, xpd_t *xpd)
{
	struct FXS_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	XPD_DBG(GENERAL, xpd, "\n");
	clean_proc(xbus, xpd);
	return 0;
}

static int FXS_card_zaptel_preregistration(xpd_t *xpd, bool on)
{
	xbus_t			*xbus;
	struct FXS_priv_data	*priv;
	int			i;

	BUG_ON(!xpd);
	xbus = xpd->xbus;
	BUG_ON(!xbus);
	priv = xpd->priv;
	BUG_ON(!priv);
	XPD_DBG(GENERAL, xpd, "%s\n", (on)?"on":"off");
#ifdef ZT_SPANSTAT_V2 
	xpd->span.spantype = "FXS";
#endif 
	for_each_line(xpd, i) {
		struct zt_chan	*cur_chan = &xpd->chans[i];

		XPD_DBG(GENERAL, xpd, "setting FXS channel %d\n", i);
		if(IS_SET(xpd->digital_outputs, i)) {
			snprintf(cur_chan->name, MAX_CHANNAME, "XPP_OUT/%02d/%1d%1d/%d",
				xbus->num, xpd->addr.unit, xpd->addr.subunit, i);
		} else if(IS_SET(xpd->digital_inputs, i)) {
			snprintf(cur_chan->name, MAX_CHANNAME, "XPP_IN/%02d/%1d%1d/%d",
				xbus->num, xpd->addr.unit, xpd->addr.subunit, i);
		} else {
			snprintf(cur_chan->name, MAX_CHANNAME, "XPP_FXS/%02d/%1d%1d/%d",
				xbus->num, xpd->addr.unit, xpd->addr.subunit, i);
		}
		cur_chan->chanpos = i + 1;
		cur_chan->pvt = xpd;
		cur_chan->sigcap = FXS_DEFAULT_SIGCAP;
	}
	for_each_line(xpd, i) {
		MARK_ON(priv, i, LED_GREEN);
		msleep(4);
	}
	return 0;
}

static int FXS_card_zaptel_postregistration(xpd_t *xpd, bool on)
{
	xbus_t			*xbus;
	struct FXS_priv_data	*priv;
	int			i;

	BUG_ON(!xpd);
	xbus = xpd->xbus;
	BUG_ON(!xbus);
	priv = xpd->priv;
	BUG_ON(!priv);
	XPD_DBG(GENERAL, xpd, "%s\n", (on)?"on":"off");
	for_each_line(xpd, i) {
		MARK_OFF(priv, i, LED_GREEN);
		msleep(2);
		MARK_OFF(priv, i, LED_RED);
		msleep(2);
	}
	restore_leds(xpd);
	return 0;
}

/*
 * Called with XPD spinlocked
 */
static void __do_mute_dtmf(xpd_t *xpd, int pos, bool muteit)
{
	LINE_DBG(SIGNAL, xpd, pos, "%s\n", (muteit) ? "MUTE" : "UNMUTE");
	if(muteit)
		BIT_SET(xpd->mute_dtmf, pos);
	else
		BIT_CLR(xpd->mute_dtmf, pos);
}

static int FXS_card_hooksig(xbus_t *xbus, xpd_t *xpd, int pos, zt_txsig_t txsig)
{
	struct FXS_priv_data	*priv;
	int			ret = 0;
	struct zt_chan		*chan = NULL;
	enum fxs_state		txhook;
	unsigned long		flags;

	LINE_DBG(SIGNAL, xpd, pos, "%s\n", txsig2str(txsig));
	priv = xpd->priv;
	BUG_ON(xpd->direction != TO_PHONE);
	if (IS_SET(xpd->digital_inputs, pos)) {
		LINE_DBG(SIGNAL, xpd, pos, "Ignoring signal sent to digital input line\n");
		return 0;
	}
	if(SPAN_REGISTERED(xpd))
		chan = &xpd->span.chans[pos];
	switch(txsig) {
		case ZT_TXSIG_ONHOOK:
			spin_lock_irqsave(&xpd->lock, flags);
			xpd->ringing[pos] = 0;
			BIT_CLR(xpd->cid_on, pos);
			BIT_CLR(priv->search_fsk_pattern, pos);
			BIT_CLR(priv->want_dtmf_events, pos);
			BIT_CLR(priv->want_dtmf_mute, pos);
			__do_mute_dtmf(xpd, pos, 0);
			__pcm_recompute(xpd, 0);	/* already spinlocked */
			spin_unlock_irqrestore(&xpd->lock, flags);
			if(IS_SET(xpd->digital_outputs, pos)) {
				LINE_DBG(SIGNAL, xpd, pos, "digital output OFF\n");
				ret = CALL_XMETHOD(RELAY_OUT, xpd->xbus, xpd, pos-8, 0);
				return ret;
			}
			if (priv->lasttxhook[pos] == FXS_LINE_OPEN) {
				/*
				 * Restore state after KEWL hangup.
				 */
				LINE_DBG(SIGNAL, xpd, pos, "KEWL STOP\n");
				linefeed_control(xbus, xpd, pos, FXS_LINE_POL_ACTIVE);
				if(IS_SET(xpd->offhook, pos))
					MARK_ON(priv, pos, LED_GREEN);
			}
			ret = CALL_XMETHOD(RING, xbus, xpd, pos, 0);			// RING off
			if (!IS_SET(xpd->offhook, pos))
				start_stop_vm_led(xbus, xpd, pos);
			txhook = priv->lasttxhook[pos];
			if(chan) {
				switch(chan->sig) {
					case ZT_SIG_EM:
					case ZT_SIG_FXOKS:
					case ZT_SIG_FXOLS:
						txhook = priv->idletxhookstate[pos];
						break;
					case ZT_SIG_FXOGS:
						txhook = FXS_LINE_TIPOPEN;
						break;
				}
			}
			ret = linefeed_control(xbus, xpd, pos, txhook);
			break;
		case ZT_TXSIG_OFFHOOK:
			txhook = priv->lasttxhook[pos];
			if(xpd->ringing[pos]) {
				BIT_SET(xpd->cid_on, pos);
				pcm_recompute(xpd, 0);
				txhook = FXS_LINE_OHTRANS;
			}
			xpd->ringing[pos] = 0;
			if(chan) {
				switch(chan->sig) {
					case ZT_SIG_EM:
						txhook = FXS_LINE_POL_ACTIVE;
						break;
					default:
						txhook = priv->idletxhookstate[pos];
						break;
				}
			}
			ret = linefeed_control(xbus, xpd, pos, txhook);
			break;
		case ZT_TXSIG_START:
			xpd->ringing[pos] = 1;
			BIT_CLR(xpd->cid_on, pos);
			BIT_CLR(priv->search_fsk_pattern, pos);
			pcm_recompute(xpd, 0);
			if(IS_SET(xpd->digital_outputs, pos)) {
				LINE_DBG(SIGNAL, xpd, pos, "%s digital output ON\n", txsig2str(txsig));
				ret = CALL_XMETHOD(RELAY_OUT, xpd->xbus, xpd, pos-8, 1);
				return ret;
			}
			ret = CALL_XMETHOD(RING, xbus, xpd, pos, 1);			// RING on
			break;
		case ZT_TXSIG_KEWL:
			LINE_DBG(SIGNAL, xpd, pos, "KEWL START\n");
			linefeed_control(xbus, xpd, pos, FXS_LINE_OPEN);
			MARK_OFF(priv, pos, LED_GREEN);
			break;
		default:
			XPD_NOTICE(xpd, "%s: Can't set tx state to %s (%d)\n",
				__FUNCTION__, txsig2str(txsig), txsig);
			ret = -EINVAL;
	}
	return ret;
}

/*
 * Private ioctl()
 * We don't need it now, since we detect vmwi via FSK patterns
 */
static int FXS_card_ioctl(xpd_t *xpd, int pos, unsigned int cmd, unsigned long arg)
{
	struct FXS_priv_data	*priv;
	xbus_t			*xbus;
	int			val;
	unsigned long		flags;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	xbus = xpd->xbus;
	BUG_ON(!xbus);
	if(!TRANSPORT_RUNNING(xbus))
		return -ENODEV;
	if (pos < 0 || pos >= xpd->channels) {
		XPD_NOTICE(xpd, "Bad channel number %d in %s(), cmd=%u\n",
			pos, __FUNCTION__, cmd);
		return -EINVAL;
	}
	
	switch (cmd) {
		case ZT_ONHOOKTRANSFER:
			if (get_user(val, (int __user *)arg))
				return -EFAULT;
			LINE_DBG(SIGNAL, xpd, pos, "ZT_ONHOOKTRANSFER (%d millis)\n", val);
			BUG_ON(pos == ALL_CHANS);
			if (IS_SET(xpd->digital_inputs | xpd->digital_outputs, pos))
				return 0;	/* Nothing to do */
			BIT_CLR(xpd->cid_on, pos);
			if(priv->lasttxhook[pos] == FXS_LINE_POL_ACTIVE) {
				priv->ohttimer[pos] = OHT_TIMER;
				priv->idletxhookstate[pos] = FXS_LINE_POL_OHTRANS;
				BIT_SET(priv->search_fsk_pattern, pos);
				pcm_recompute(xpd, priv->search_fsk_pattern);
			}
			if(!IS_SET(xpd->offhook, pos))
				start_stop_vm_led(xbus, xpd, pos);
			return 0;
		case ZT_TONEDETECT:
			if (get_user(val, (int __user *)arg))
				return -EFAULT;
			LINE_DBG(SIGNAL, xpd, pos, "ZT_TONEDETECT: %s %s (dtmf_detection=%s)\n",
					(val & ZT_TONEDETECT_ON) ? "ON" : "OFF",
					(val & ZT_TONEDETECT_MUTE) ? "MUTE" : "NO-MUTE",
					(dtmf_detection ? "YES" : "NO"));
			if(!dtmf_detection) {
				spin_lock_irqsave(&xpd->lock, flags);
				if(IS_SET(priv->want_dtmf_events, pos)) {
					/* Detection mode changed: Disable DTMF interrupts */
					SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x17, 0);
				}
				BIT_CLR(priv->want_dtmf_events, pos);
				BIT_CLR(priv->want_dtmf_mute, pos);
				__do_mute_dtmf(xpd, pos, 0);
				__pcm_recompute(xpd, 0);	/* already spinlocked */
				spin_unlock_irqrestore(&xpd->lock, flags);
				return -ENOTTY;
			}
			/*
			 * During natively bridged calls, Asterisk
			 * will request one of the sides to stop sending
			 * dtmf events. Check the requested state.
			 */
			spin_lock_irqsave(&xpd->lock, flags);
			if(val & ZT_TONEDETECT_ON) {
				if(!IS_SET(priv->want_dtmf_events, pos)) {
					/* Detection mode changed: Enable DTMF interrupts */
					LINE_DBG(SIGNAL, xpd, pos,
						"ZT_TONEDETECT: Enable Hardware DTMF\n");
					SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x17, 1);
				}
				BIT_SET(priv->want_dtmf_events, pos);
			} else {
				if(IS_SET(priv->want_dtmf_events, pos)) {
					/* Detection mode changed: Disable DTMF interrupts */
					LINE_DBG(SIGNAL, xpd, pos,
						"ZT_TONEDETECT: Disable Hardware DTMF\n");
					SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x17, 0);
				}
				BIT_CLR(priv->want_dtmf_events, pos);
			}
			if(val & ZT_TONEDETECT_MUTE) {
				BIT_SET(priv->want_dtmf_mute, pos);
			} else {
				BIT_CLR(priv->want_dtmf_mute, pos);
				__do_mute_dtmf(xpd, pos, 0);
				__pcm_recompute(xpd, 0);
			}
			spin_unlock_irqrestore(&xpd->lock, flags);
			return 0;
		case ZT_SETPOLARITY:
			if (get_user(val, (int __user *)arg))
				return -EFAULT;
			/* Can't change polarity while ringing or when open */
			if (priv->lasttxhook[pos] == FXS_LINE_RING || priv->lasttxhook[pos] == FXS_LINE_OPEN) {
				LINE_ERR(xpd, pos, "ZT_SETPOLARITY: %s Cannot change when lasttxhook=0x%X\n",
						(val)?"ON":"OFF", priv->lasttxhook[pos]);
				return -EINVAL;
			}
			LINE_DBG(SIGNAL, xpd, pos, "ZT_SETPOLARITY: %s\n", (val)?"ON":"OFF");
			if ((val && !reversepolarity) || (!val && reversepolarity))
				priv->lasttxhook[pos] |= FXS_LINE_RING;
			else
				priv->lasttxhook[pos] &= ~FXS_LINE_RING;
			linefeed_control(xbus, xpd, pos, priv->lasttxhook[pos]);
			return 0;
#ifdef	ZT_VMWI
		case ZT_VMWI:		/* message-waiting led control */
			if (get_user(val, (int __user *)arg))
				return -EFAULT;
			if(!vmwi_ioctl) {
				LINE_NOTICE(xpd, pos, "Got ZT_VMWI notification but vmwi_ioctl parameter is off. Ignoring.\n");
				return 0;
			}
			/* Digital inputs/outputs don't have VM leds */
			if (IS_SET(xpd->digital_inputs | xpd->digital_outputs, pos))
				return 0;
			if (val)
				BIT_SET(xpd->msg_waiting, pos);
			else
				BIT_CLR(xpd->msg_waiting, pos);
			return 0;
#endif
		default:
			report_bad_ioctl(THIS_MODULE->name, xpd, pos, cmd);
	}
	return -ENOTTY;
}

static int set_vm_led_mode(xbus_t *xbus, xpd_t *xpd, int pos, int on)
{
	int	ret = 0;
	BUG_ON(!xbus);
	BUG_ON(!xpd);

	LINE_DBG(SIGNAL, xpd, pos, "%s%s\n", (on)?"ON":"OFF", (vmwineon)?"":" (Ignored)");
	if (!vmwineon)
		return 0;
	if (on) {
		/* A write to register 0x40 will now turn on/off the VM led */
		ret += SLIC_INDIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x16, 0xE8, 0x03);
		ret += SLIC_INDIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x15, 0xEF, 0x7B);
		ret += SLIC_INDIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x14, 0x9F, 0x00);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x22, 0x19);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x4A, 0x34);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x30, 0xE0);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x31, 0x01);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x32, 0xF0);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x33, 0x05);
		ret += SLIC_INDIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x1D, 0x00, 0x46);
	} else {
		/* A write to register 0x40 will now turn on/off the ringer */
		ret += SLIC_INDIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x16, 0x00, 0x00);
		ret += SLIC_INDIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x15, 0x60, 0x01);
		ret += SLIC_INDIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x14, 0xF0, 0x7E);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x22, 0x00);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x4A, 0x34);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x30, 0x00);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x31, 0x00);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x32, 0x00);
		ret += SLIC_DIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x33, 0x00);
		ret += SLIC_INDIRECT_REQUEST(xbus, xpd, pos, SLIC_WRITE, 0x1D, 0x00, 0x36);
	}

	return (ret ? -EPROTO : 0);
}

static void start_stop_vm_led(xbus_t *xbus, xpd_t *xpd, lineno_t pos)
{
	struct FXS_priv_data	*priv;
	bool		on;

	BUG_ON(!xpd);
	if (!vmwineon || IS_SET(xpd->digital_outputs | xpd->digital_inputs, pos))
		return;
	priv = xpd->priv;
	on = IS_SET(xpd->msg_waiting, pos);
	LINE_DBG(SIGNAL, xpd, pos, "%s\n", (on)?"ON":"OFF");
	set_vm_led_mode(xbus, xpd, pos, on);
	do_chan_power(xbus, xpd, pos, on);
	linefeed_control(xbus, xpd, pos, (on) ? FXS_LINE_RING : priv->idletxhookstate[pos]);
}

static int FXS_card_open(xpd_t *xpd, lineno_t chan)
{
	struct FXS_priv_data	*priv;
	bool			is_offhook;

	BUG_ON(!xpd);
	priv = xpd->priv;
	is_offhook = IS_SET(xpd->offhook, chan);
	LINE_DBG(GENERAL, xpd, chan, "(is %shook)\n", (is_offhook)?"off":"on");
	/*
	 * Delegate updating zaptel to FXS_card_tick():
	 *   The problem is that zt_hooksig() is spinlocking the channel and
	 *   we are called by zaptel with the spinlock already held on the
	 *   same channel.
	 */
	BIT_SET(priv->update_offhook_state, chan);
	return 0;
}

static int FXS_card_close(xpd_t *xpd, lineno_t chan)
{
	struct FXS_priv_data	*priv;

	BUG_ON(!xpd);
	LINE_DBG(GENERAL, xpd, chan, "\n");
	priv = xpd->priv;
	priv->idletxhookstate[chan] = FXS_LINE_POL_ACTIVE;
	return 0;
}

#ifdef	POLL_DIGITAL_INPUTS
/*
 * INPUT polling is done via SLIC register 0x06 (same as LEDS):
 *         7     6     5     4     3     2     1     0
 * 	+-----+-----+-----+-----+-----+-----+-----+-----+
 * 	| I1  | I3  |     |     | I2  | I4  |     |     |
 * 	+-----+-----+-----+-----+-----+-----+-----+-----+
 *
 */
static int	input_channels[] = { 6, 7, 2, 3 };	// Slic numbers of input relays

static void poll_inputs(xpd_t *xpd)
{
	int	i;

	BUG_ON(xpd->xbus_idx != 0);	// Only unit #0 has digital inputs
	for(i = 0; i < ARRAY_SIZE(input_channels); i++) {
		byte	pos = input_channels[i];

		SLIC_DIRECT_REQUEST(xpd->xbus, xpd, pos, SLIC_READ, 0x06, 0);
	}
}
#endif

static void handle_linefeed(xpd_t *xpd)
{
	struct FXS_priv_data	*priv;
	int			i;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	for_each_line(xpd, i) {
		if (priv->lasttxhook[i] == FXS_LINE_RING) {
			/* RINGing, prepare for OHT */
			priv->ohttimer[i] = OHT_TIMER;
			priv->idletxhookstate[i] = FXS_LINE_POL_OHTRANS;
		} else {
			if (priv->ohttimer[i]) {
				priv->ohttimer[i]--;
				if (!priv->ohttimer[i]) {
					priv->idletxhookstate[i] = FXS_LINE_POL_ACTIVE;
					if (priv->lasttxhook[i] == FXS_LINE_POL_OHTRANS) {
						enum fxs_state	txhook = FXS_LINE_POL_ACTIVE;
						/* Apply the change if appropriate */
						BIT_CLR(xpd->cid_on, i);
						BIT_CLR(priv->search_fsk_pattern, i);
						pcm_recompute(xpd, 0);
						linefeed_control(xpd->xbus, xpd, i, txhook);
					}
				}
			}
		}
	}
}

/*
 * Optimized memcmp() like function. Only test for equality (true/false).
 * This optimization reduced the detect_vmwi() runtime by a factor of 3.
 */
static inline bool mem_equal(const char a[], const char b[], size_t len)
{
	int	i;

	for(i = 0; i < len; i++)
		if(a[i] != b[i])
			return 0;
	return 1;
}

/*
 * Detect Voice Mail Waiting Indication
 */
static void detect_vmwi(xpd_t *xpd)
{
	struct FXS_priv_data	*priv;
	xbus_t			*xbus;
	static const byte	FSK_COMMON_PATTERN[] = { 0xA8, 0x49, 0x22, 0x3B, 0x9F, 0xFF, 0x1F, 0xBB };
	static const byte	FSK_ON_PATTERN[] = { 0xA2, 0x2C, 0x1F, 0x2C, 0xBB, 0xA1, 0xA5, 0xFF };
	static const byte	FSK_OFF_PATTERN[] = { 0xA2, 0x2C, 0x28, 0xA5, 0xB1, 0x21, 0x49, 0x9F };
	int			i;

	BUG_ON(!xpd);
	xbus = xpd->xbus;
	priv = xpd->priv;
	BUG_ON(!priv);
	for_each_line(xpd, i) {
		struct zt_chan	*chan = &xpd->span.chans[i];
		byte		*writechunk = chan->writechunk;

		if(IS_SET(xpd->offhook | xpd->cid_on | xpd->digital_inputs | xpd->digital_outputs, i))
			continue;
#if 0
		if(writechunk[0] != 0x7F && writechunk[0] != 0) {
			int	j;

			LINE_DBG(GENERAL, xpd, pos, "MSG:");
			for(j = 0; j < ZT_CHUNKSIZE; j++) {
				if(print_dbg)
					printk(" %02X", writechunk[j]);
			}
			if(print_dbg)
				printk("\n");
		}
#endif
		if(unlikely(mem_equal(writechunk, FSK_COMMON_PATTERN, ZT_CHUNKSIZE)))
			BIT_SET(priv->found_fsk_pattern, i);
		else if(unlikely(IS_SET(priv->found_fsk_pattern, i))) {
			BIT_CLR(priv->found_fsk_pattern, i);
			if(unlikely(mem_equal(writechunk, FSK_ON_PATTERN, ZT_CHUNKSIZE))) {
				LINE_DBG(SIGNAL, xpd, i, "MSG WAITING ON\n");
				BIT_SET(xpd->msg_waiting, i);
				start_stop_vm_led(xbus, xpd, i);
			} else if(unlikely(mem_equal(writechunk, FSK_OFF_PATTERN, ZT_CHUNKSIZE))) {
				LINE_DBG(SIGNAL, xpd, i, "MSG WAITING OFF\n");
				BIT_CLR(xpd->msg_waiting, i);
				start_stop_vm_led(xbus, xpd, i);
			} else {
				int	j;

				LINE_NOTICE(xpd, i, "MSG WAITING Unexpected:");
				for(j = 0; j < ZT_CHUNKSIZE; j++) {
					printk(" %02X", writechunk[j]);
				}
				printk("\n");
			}
		}
	}
}

static int FXS_card_tick(xbus_t *xbus, xpd_t *xpd)
{
	struct FXS_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
#ifdef	POLL_DIGITAL_INPUTS
	if(poll_digital_inputs && xpd->xbus_idx == 0) {
		if((xpd->timer_count % poll_digital_inputs) == 0)
			poll_inputs(xpd);
	}
#endif
	handle_fxs_leds(xpd);
	handle_linefeed(xpd);
	if(priv->update_offhook_state) {	/* set in FXS_card_open() */
		int	i;

		for_each_line(xpd, i) {
			if(!IS_SET(priv->update_offhook_state, i))
				continue;
			/*
			 * Update LEDs and zaptel with current state of line.
			 */
			if(IS_SET(xpd->offhook, i)) {
				LINE_NOTICE(xpd, i, "Already offhook during open. OK.\n");
				MARK_ON(priv, i, LED_GREEN);
				update_line_status(xpd, i, 1);
			} else {
				MARK_OFF(priv, i, LED_GREEN);
				update_line_status(xpd, i, 0);
			}
			BIT_CLR(priv->update_offhook_state, i);
		}
	}
	if(SPAN_REGISTERED(xpd)) {
		if(vmwineon && !vmwi_ioctl)
			detect_vmwi(xpd);	/* Detect via FSK modulation */
	}
	return 0;
}

/*---------------- FXS: HOST COMMANDS -------------------------------------*/

/* 0x0F */ HOSTCMD(FXS, XPD_STATE, bool on)
{
	int		i;
	enum fxs_state	value = (on) ? FXS_LINE_POL_ACTIVE : FXS_LINE_OPEN;
	unsigned long		flags;
	struct FXS_priv_data	*priv;

	BUG_ON(!xbus);
	BUG_ON(!xpd);
	priv = xpd->priv;
	spin_lock_irqsave(&xpd->lock, flags);
	XPD_DBG(GENERAL, xpd, "%s\n", (on)?"on":"off");
	for_each_line(xpd, i)
		linefeed_control(xbus, xpd, i, value);
	if(on) {
		MARK_ON(priv, ALL_CHANS, LED_GREEN);
	} else {
		MARK_OFF(priv, ALL_CHANS, LED_GREEN);
	}
	spin_unlock_irqrestore(&xpd->lock, flags);
	return 0;
}

/* 0x0F */ HOSTCMD(FXS, RING, lineno_t chan, bool on)
{
	int			ret = 0;
	struct FXS_priv_data	*priv;
	enum fxs_state		value = (on) ? FXS_LINE_RING : FXS_LINE_POL_ACTIVE;

	BUG_ON(!xbus);
	BUG_ON(!xpd);
	LINE_DBG(SIGNAL, xpd, chan, "%s\n", (on)?"on":"off");
	priv = xpd->priv;
	set_vm_led_mode(xbus, xpd, chan, 0);
	do_chan_power(xbus, xpd, chan, on);		// Power up (for ring)
	ret = linefeed_control(xbus, xpd, chan, value);
	if(on) {
		MARK_BLINK(priv, chan, LED_GREEN, LED_BLINK_RING);
	} else {
		if(IS_BLINKING(priv, chan, LED_GREEN))
			MARK_BLINK(priv, chan, LED_GREEN, 0);
	}
	return ret;
}

/* 0x0F */ HOSTCMD(FXS, RELAY_OUT, byte which, bool on)
{
	int		value;
	int		relay_channels[] = { 0, 4 };

	BUG_ON(!xbus);
	BUG_ON(!xpd);
	XPD_DBG(SIGNAL, xpd, "RELAY_OUT: which=%d -- %s\n", which, (on) ? "on" : "off");
	which = which % ARRAY_SIZE(relay_channels);
	value = BIT(2) | BIT(3);
	value |= ((BIT(5) | BIT(6) | BIT(7)) & ~led_register_mask[OUTPUT_RELAY]);
	if(on)
		value |= led_register_vals[OUTPUT_RELAY];
	return SLIC_DIRECT_REQUEST(xbus, xpd, relay_channels[which], SLIC_WRITE, 0x06, value);
}

/*---------------- FXS: Astribank Reply Handlers --------------------------*/

HANDLER_DEF(FXS, SIG_CHANGED)
{
	xpp_line_t		sig_status = RPACKET_FIELD(pack, FXS, SIG_CHANGED, sig_status);
	xpp_line_t		sig_toggles = RPACKET_FIELD(pack, FXS, SIG_CHANGED, sig_toggles);
	struct FXS_priv_data	*priv;
	int			i;
	unsigned long		flags;

	BUG_ON(!xpd);
	BUG_ON(xpd->direction != TO_PHONE);
	priv = xpd->priv;
	XPD_DBG(SIGNAL, xpd, "(PHONE) sig_toggles=0x%04X sig_status=0x%04X\n", sig_toggles, sig_status);
#if 0
	Is this needed?
	for_each_line(xpd, i) {
		if(IS_SET(sig_toggles, i))
			do_chan_power(xpd->xbus, xpd, BIT(i), 0);		// Power down (prevent overheating!!!)
	}
#endif
	spin_lock_irqsave(&xpd->lock, flags);
	for_each_line(xpd, i) {
		if(IS_SET(xpd->digital_outputs, i) || IS_SET(xpd->digital_inputs, i))
			continue;
		if(IS_SET(sig_toggles, i)) {
			xpd->ringing[i] = 0;		/* No more ringing... */
#ifdef	WITH_METERING
			metering_gen(xpd, i, 0);	/* Stop metering... */
#endif
			MARK_BLINK(priv, i, LED_GREEN, 0);
			if(IS_SET(sig_status, i)) {
				LINE_DBG(SIGNAL, xpd, i, "OFFHOOK\n");
				MARK_ON(priv, i, LED_GREEN);
				update_line_status(xpd, i, 1);
			} else {
				LINE_DBG(SIGNAL, xpd, i, "ONHOOK\n");
				MARK_OFF(priv, i, LED_GREEN);
				update_line_status(xpd, i, 0);
			}
		}
	}
	__pcm_recompute(xpd, 0);	/* in a spinlock */
	spin_unlock_irqrestore(&xpd->lock, flags);
	return 0;
}

#ifdef	POLL_DIGITAL_INPUTS
static void process_digital_inputs(xpd_t *xpd, const reg_cmd_t *info)
{
	int		i;
	bool		offhook = (REG_FIELD(info, data_low) & 0x1) == 0;
	xpp_line_t	lines = BIT(REG_FIELD(info, chipsel));

	/* Map SLIC number into line number */
	for(i = 0; i < ARRAY_SIZE(input_channels); i++) {
		int		channo = input_channels[i];
		int		newchanno;

		if(IS_SET(lines, channo)) {
			newchanno = xpd->channels - LINES_DIGI_INP + i;
			BIT_CLR(lines, channo);
			BIT_SET(lines, newchanno);
			xpd->ringing[newchanno] = 0;			// Stop ringing. No leds for digital inputs.
			if(offhook && !IS_SET(xpd->offhook, newchanno)) {		// OFFHOOK
				LINE_DBG(SIGNAL, xpd, newchanno, "OFFHOOK\n");
				update_line_status(xpd, newchanno, 1);
			} else if(!offhook && IS_SET(xpd->offhook, newchanno)) {	// ONHOOK
				LINE_DBG(SIGNAL, xpd, newchanno, "ONHOOK\n");
				update_line_status(xpd, newchanno, 0);
			}
		}
	}
}
#endif

static const char dtmf_digits[] = {
	'1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '*', '#', 'A', 'B', 'C', 'D'
};

/*
 * This function is called with spinlocked XPD
 */
static void process_dtmf(xpd_t *xpd, xpp_line_t lines, byte val)
{
	int			i;
	byte			digit;
	bool			is_down = val & 0x10;
	struct FXS_priv_data	*priv;

	if(!dtmf_detection)
		return;
	priv = xpd->priv;
	val &= 0xF;
	if(val <= 0) {
		if(is_down)
			XPD_NOTICE(xpd, "Bad DTMF value %d. Ignored\n", val);
		return;
	}
	val--;
	digit = dtmf_digits[val];
	for_each_line(xpd, i) {
		if(IS_SET(lines, i)) {
			int		event = (is_down) ? ZT_EVENT_DTMFDOWN : ZT_EVENT_DTMFUP;
			bool		want_mute = IS_SET(priv->want_dtmf_mute, i);
			bool		want_event = IS_SET(priv->want_dtmf_events, i);

			if(want_event) {
				LINE_DBG(SIGNAL, xpd, i,
					"DTMF digit %s (val=%d) '%c' (want_mute=%s)\n",
					(is_down)?"DOWN":"UP", val, digit,
					(want_mute) ? "yes" : "no");
			} else {
				LINE_DBG(SIGNAL, xpd, i,
					"Ignored DTMF digit %s '%c'\n",
					(is_down)?"DOWN":"UP", digit);
			}
			/*
			 * FIXME: we currently don't use the want_dtmf_mute until
			 * we are sure about the logic in Asterisk native bridging.
			 * Meanwhile, simply mute it on button press.
			 */
			if(is_down && want_mute)
				__do_mute_dtmf(xpd, i, 1);
			else
				__do_mute_dtmf(xpd, i, 0);
			__pcm_recompute(xpd, 0);	/* XPD is locked */
			if(want_event) 
				zt_qevent_lock(&xpd->chans[i], event | digit);
			break;
		}
	}
}

static int FXS_card_register_reply(xbus_t *xbus, xpd_t *xpd, reg_cmd_t *info)
{
	unsigned long		flags;
	struct FXS_priv_data	*priv;
	byte			regnum;
	bool			indirect;

	spin_lock_irqsave(&xpd->lock, flags);
	priv = xpd->priv;
	BUG_ON(!priv);
	indirect = (REG_FIELD(info, regnum) == 0x1E);
	regnum = (indirect) ? REG_FIELD(info, subreg) : REG_FIELD(info, regnum);
	XPD_DBG(REGS, xpd, "%s reg_num=0x%X, dataL=0x%X dataH=0x%X\n",
			(indirect)?"I":"D",
			regnum, REG_FIELD(info, data_low), REG_FIELD(info, data_high));
	if(!SPAN_REGISTERED(xpd))
		goto out;
#ifdef	POLL_DIGITAL_INPUTS
	/*
	 * Process digital inputs polling results
	 */
	if(xpd->xbus_idx == 0 && !indirect && regnum == 0x06)
		process_digital_inputs(xpd, info);
#endif
	if(!indirect && regnum == SLIC_REG_DTMF) {
		byte		val = REG_FIELD(info, data_low);
		xpp_line_t	lines = BIT(REG_FIELD(info, chipsel));

#if 0
		XPD_DBG(SIGNAL, xpd, "DTMF result lines=0x%04X val=%d\n",
				lines, val);
#endif
		process_dtmf(xpd, lines, val);
	}
out:
	/* Update /proc info only if reply relate to the last slic read request */
	if(
			REG_FIELD(&xpd->requested_reply, regnum) == REG_FIELD(info, regnum) &&
			REG_FIELD(&xpd->requested_reply, do_subreg) == REG_FIELD(info, do_subreg) &&
			REG_FIELD(&xpd->requested_reply, subreg) == REG_FIELD(info, subreg)) {
		xpd->last_reply = *info;
	}
	spin_unlock_irqrestore(&xpd->lock, flags);
	return 0;
}

static xproto_table_t PROTO_TABLE(FXS) = {
	.owner = THIS_MODULE,
	.entries = {
		/*	Prototable	Card	Opcode		*/
		XENTRY(	FXS,		FXS,	SIG_CHANGED	),
	},
	.name = "FXS",
	.type = XPD_TYPE_FXS,
	.xops = {
		.card_new	= FXS_card_new,
		.card_init	= FXS_card_init,
		.card_remove	= FXS_card_remove,
		.card_zaptel_preregistration	= FXS_card_zaptel_preregistration,
		.card_zaptel_postregistration	= FXS_card_zaptel_postregistration,
		.card_hooksig	= FXS_card_hooksig,
		.card_tick	= FXS_card_tick,
		.card_pcm_fromspan	= generic_card_pcm_fromspan,
		.card_pcm_tospan	= generic_card_pcm_tospan,
		.card_open	= FXS_card_open,
		.card_close	= FXS_card_close,
		.card_ioctl	= FXS_card_ioctl,
		.card_register_reply	= FXS_card_register_reply,

		.RING		= XPROTO_CALLER(FXS, RING),
		.RELAY_OUT	= XPROTO_CALLER(FXS, RELAY_OUT),
		.XPD_STATE	= XPROTO_CALLER(FXS, XPD_STATE),
	},
	.packet_is_valid = fxs_packet_is_valid,
	.packet_dump = fxs_packet_dump,
};

static bool fxs_packet_is_valid(xpacket_t *pack)
{
	const xproto_entry_t	*xe;

	// DBG(GENERAL, "\n");
	xe = xproto_card_entry(&PROTO_TABLE(FXS), XPACKET_OP(pack));
	return xe != NULL;
}

static void fxs_packet_dump(const char *msg, xpacket_t *pack)
{
	DBG(GENERAL, "%s\n", msg);
}

/*------------------------- SLIC Handling --------------------------*/

static int proc_fxs_info_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int			len = 0;
	unsigned long		flags;
	xpd_t			*xpd = data;
	struct FXS_priv_data	*priv;
	int			i;
	int			led;

	if(!xpd)
		return -ENODEV;
	spin_lock_irqsave(&xpd->lock, flags);
	priv = xpd->priv;
	BUG_ON(!priv);
	len += sprintf(page + len, "%-8s %-10s %-10s %-10s\n",
			"Channel",
			"idletxhookstate",
			"lasttxhook",
			"ohttimer"
			);
	for_each_line(xpd, i) {
		char	pref;

		if(IS_SET(xpd->digital_outputs, i))
			pref = 'O';
		else if(IS_SET(xpd->digital_inputs, i))
			pref = 'I';
		else
			pref = ' ';
		len += sprintf(page + len, "%c%7d %10d %10d %10d\n",
				pref,
				i,
				priv->idletxhookstate[i],
				priv->lasttxhook[i],
				priv->ohttimer[i]
			      );
	}
	len += sprintf(page + len, "\n");
	for(led = 0; led < NUM_LEDS; led++) {
		len += sprintf(page + len, "LED #%d", led);
		len += sprintf(page + len, "\n\t%-17s: ", "ledstate");
		for_each_line(xpd, i) {
			if(!IS_SET(xpd->digital_outputs, i) && !IS_SET(xpd->digital_inputs, i))
				len += sprintf(page + len, "%d ", IS_SET(priv->ledstate[led], i));
		}
		len += sprintf(page + len, "\n\t%-17s: ", "ledcontrol");
		for_each_line(xpd, i) {
			if(!IS_SET(xpd->digital_outputs, i) && !IS_SET(xpd->digital_inputs, i))
				len += sprintf(page + len, "%d ", IS_SET(priv->ledcontrol[led], i));
		}
		len += sprintf(page + len, "\n\t%-17s: ", "led_counter");
		for_each_line(xpd, i) {
			if(!IS_SET(xpd->digital_outputs, i) && !IS_SET(xpd->digital_inputs, i))
				len += sprintf(page + len, "%d ", LED_COUNTER(priv,i,led));
		}
		len += sprintf(page + len, "\n");
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
	unsigned		data_low = 0;
	unsigned		data_high = 0;
	char			op;		/* [W]rite, [R]ead */
	char			reg_type;	/* [D]irect, [I]ndirect */
	int			reg_num;
	int			elements;
	bool			writing;
	char			*p;
	reg_cmd_t		regcmd;
	xbus_t			*xbus;
	int			ret = -EINVAL;

	BUG_ON(!xpd);
	xbus = xpd->xbus;
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
	elements = sscanf(cmdline, "%d %c%c %x %x %x",
			&chipsel,
			&op, &reg_type, &reg_num,
			&data_low,
			&data_high);
	XPD_DBG(REGS, xpd, "'%s': %d %c%c %02X %02X %02X\n", cmdline, chipsel, op, reg_type, reg_num, data_low, data_high);
	if(elements < 4) {	// At least: chipsel, op, reg_type, reg_num
		ERR("Not enough arguments: (%d args) '%s'\n", elements, cmdline);
		goto out;
	}
	if(!VALID_CHIPSEL(chipsel)) {
		ERR("Bad chipsel number: %d\n", chipsel);
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
	switch(reg_type) {
		case 'I':
			REG_FIELD(&regcmd, do_subreg) = 1;
			REG_FIELD(&regcmd, regnum) = 0x1E;		// FIXME: card dependent...
			REG_FIELD(&regcmd, subreg) = reg_num;
			break;
		case 'D':
			REG_FIELD(&regcmd, do_subreg) = 0;
			REG_FIELD(&regcmd, regnum) = reg_num;
			REG_FIELD(&regcmd, subreg) = 0;
			break;
		default:
			ERR("Unkown register type '%c'\n", reg_type);
			goto out;
	}
	if(
			(op == 'W' && reg_type == 'D' && elements != 5) ||
			(op == 'W' && reg_type == 'I' && elements != 6) ||
			(op == 'R' && reg_type == 'D' && elements != 4) ||
			(op == 'R' && reg_type == 'I' && elements != 4)
	  ) {
		ERR("%s: '%s' (%d elements): %d %c%c %02X %02X %02X\n", __FUNCTION__,
				cmdline, elements,
				chipsel, op, reg_type, reg_num, data_low, data_high);
		goto out;
	}
	regcmd.bytes = sizeof(regcmd) - 1;
	REG_FIELD(&regcmd, data_low) = data_low;
	REG_FIELD(&regcmd, data_high) = data_high;
	REG_FIELD(&regcmd, read_request) = writing;
	xpd->requested_reply = regcmd;
	if(print_dbg)
		dump_reg_cmd("FXS", &regcmd, 1);
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
	byte			regnum;
	bool			indirect;

	if(!xpd)
		return -ENODEV;
	spin_lock_irqsave(&xpd->lock, flags);
	info = &xpd->last_reply;
	indirect = (REG_FIELD(info, regnum) == 0x1E);
	regnum = (indirect) ? REG_FIELD(info, subreg) : REG_FIELD(info, regnum);
	len += sprintf(page + len, "# Writing bad data into this file may damage your hardware!\n");
	len += sprintf(page + len, "# Consult firmware docs first\n");
	len += sprintf(page + len, "#\n");
	len += sprintf(page + len, "#CH\tD/I\tReg.\tDL DH\n");
	len += sprintf(page + len, "%2d\tR%c\t%02X\t%02X %02X\n",
			REG_FIELD(info, chipsel),
			(indirect)?'I':'D',
			regnum, REG_FIELD(info, data_low), REG_FIELD(info, data_high));
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

#ifdef	WITH_METERING
static int proc_xpd_metering_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	xpd_t		*xpd = data;
	char		buf[MAX_PROC_WRITE];
	lineno_t	chan;
	int		num;
	int		ret;

	if(!xpd)
		return -ENODEV;
	if(count >= MAX_PROC_WRITE - 1) {
		XPD_ERR(xpd, "Metering string too long (%lu)\n", count);
		return -EINVAL;
	}
	if(copy_from_user(&buf, buffer, count))
		return -EFAULT;
	buf[count] = '\0';
	ret = sscanf(buf, "%d", &num);
	if(ret != 1) {
		XPD_ERR(xpd, "Metering value should be number. Got '%s'\n", buf);
		return -EINVAL;
	}
	chan = num;
	if(chan != ALL_CHANS && chan > xpd->channels) {
		XPD_ERR(xpd, "Metering tone: bad channel number %d\n", chan);
		return -EINVAL;
	}
	if((ret = metering_gen(xpd, chan, 1)) < 0) {
		XPD_ERR(xpd, "Failed sending metering tone\n");
		return ret;
	}
	return count;
}
#endif

int __init card_fxs_startup(void)
{
	INFO("revision %s\n", XPP_VERSION);
#ifdef	POLL_DIGITAL_INPUTS
	INFO("FEATURE: with DIGITAL INPUTS support (polled every %d msec)\n",
			poll_digital_inputs);
#else
	INFO("FEATURE: without DIGITAL INPUTS support\n");
#endif
#ifdef	ZT_VMWI
	INFO("FEATURE: ZT_VMWI\n");
#else
	INFO("FEATURE: NO ZT_VMWI\n");
#endif
#ifdef	WITH_METERING
	INFO("FEATURE: WITH METERING Generation\n");
#else
	INFO("FEATURE: NO METERING Generation\n");
#endif
	xproto_register(&PROTO_TABLE(FXS));
	return 0;
}

void __exit card_fxs_cleanup(void)
{
	xproto_unregister(&PROTO_TABLE(FXS));
}

MODULE_DESCRIPTION("XPP FXS Card Driver");
MODULE_AUTHOR("Oron Peled <oron@actcom.co.il>");
MODULE_LICENSE("GPL");
MODULE_VERSION(XPP_VERSION);
MODULE_ALIAS_XPD(XPD_TYPE_FXS);

module_init(card_fxs_startup);
module_exit(card_fxs_cleanup);
