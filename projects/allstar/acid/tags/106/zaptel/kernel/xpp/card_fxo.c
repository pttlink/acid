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

static const char rcsid[] = "$Id: card_fxo.c 3957 2008-03-07 00:45:53Z tzafrir $";

DEF_PARM(int, print_dbg, 0, 0644, "Print DBG statements");
DEF_PARM(uint, poll_battery_interval, 500, 0644, "Poll battery interval in milliseconds (0 - disable)");
DEF_PARM(uint, poll_power_denial_interval, 40, 0644, "Power denial detection poll interval in milliseconds (0 - disable)");
#ifdef	WITH_METERING
DEF_PARM(uint, poll_metering_interval, 500, 0644, "Poll metering interval in milliseconds (0 - disable)");
#endif
DEF_PARM(int, ring_debounce, 50, 0644, "Number of ticks to debounce a false RING indication");
DEF_PARM(int, caller_id_style, 0, 0444, "Caller-Id detection style: 0 - [BELL], 1 - [BT], 2 - [PASS]");

enum cid_style {
	CID_STYLE_BELL		= 0,	/* E.g: US (Bellcore) */
	CID_STYLE_ETSI_POLREV	= 1,	/* E.g: UK (British Telecom) */
	CID_STYLE_PASS_ALWAYS	= 2,	/* E.g: DK */
};

/* Signaling is opposite (fxs signalling for fxo card) */
#if 1
#define	FXO_DEFAULT_SIGCAP	(ZT_SIG_FXSKS | ZT_SIG_FXSLS)
#else
#define	FXO_DEFAULT_SIGCAP	(ZT_SIG_SF)
#endif

enum fxo_leds {
	LED_GREEN,
};

#define	NUM_LEDS		1
#define	DELAY_UNTIL_DIALTONE	3000

/*
 * Minimum duration for polarity reversal detection (in ticks)
 * Should be longer than the time to detect a ring, so voltage
 * fluctuation during ring won't trigger false detection.
 */
#define	POLREV_THRESHOLD	200
#define	BAT_THRESHOLD		3
#define	BAT_DEBOUNCE		1000	/* compensate for battery voltage fluctuation (in ticks) */
#define	POWER_DENIAL_CURRENT	3
#define	POWER_DENIAL_TIME	80	/* ticks */
#define	POWER_DENIAL_SAFEZONE	100	/* ticks */
#define	POWER_DENIAL_DELAY	2500	/* ticks */

/* Shortcuts */
#define	DAA_WRITE	1
#define	DAA_READ	0
#define	DAA_DIRECT_REQUEST(xbus,xpd,chipsel,writing,reg,dL)	\
	xpp_register_request((xbus), (xpd), (chipsel), (writing), 0, (reg), 0, (dL), 0)

#define	VALID_CHIPSEL(x)	(((chipsel) >= 0 && (chipsel) <= 7) || (chipsel) == ALL_CHANS)

/*---------------- FXO Protocol Commands ----------------------------------*/

static /* 0x0F */ DECLARE_CMD(FXO, XPD_STATE, bool on);
static /* 0x0F */ DECLARE_CMD(FXO, RING, lineno_t chan, bool on);
static /* 0x0F */ DECLARE_CMD(FXO, RELAY_OUT, byte which, bool on);

static bool fxo_packet_is_valid(xpacket_t *pack);
static void fxo_packet_dump(const char *msg, xpacket_t *pack);
static int proc_fxo_info_read(char *page, char **start, off_t off, int count, int *eof, void *data);
#ifdef	WITH_METERING
static int proc_xpd_metering_read(char *page, char **start, off_t off, int count, int *eof, void *data);
#endif
static int proc_xpd_register_read(char *page, char **start, off_t off, int count, int *eof, void *data);
static int proc_xpd_register_write(struct file *file, const char __user *buffer, unsigned long count, void *data);
static int handle_register_command(xpd_t *xpd, char *cmdline);
static void zap_report_battery(xpd_t *xpd, lineno_t chan);

#define	PROC_REGISTER_FNAME	"slics"
#define	PROC_FXO_INFO_FNAME	"fxo_info"
#ifdef	WITH_METERING
#define	PROC_METERING_FNAME	"metering_read"
#endif

#define	DAA_REG_RING		0x05
#define	DAA_REG_METERING	0x11	/* 17 */
#define	DAA_REG_CURRENT		0x1C	/* 28 */
#define	DAA_REG_VBAT		0x1D	/* 29 */

enum battery_state {
	BATTERY_UNKNOWN	= 0,
	BATTERY_ON		= 1,
	BATTERY_OFF		= -1
};

enum polarity_state {
	POL_UNKNOWN	= 0,
	POL_POSITIVE	= 1,
	POL_NEGATIVE	= -1
};

struct FXO_priv_data {
	struct proc_dir_entry	*regfile;
#ifdef	WITH_METERING
	struct proc_dir_entry	*meteringfile;
#endif
	struct proc_dir_entry	*fxo_info;
	uint			poll_counter;
	signed char		battery_voltage[CHANNELS_PERXPD];
	signed char		battery_current[CHANNELS_PERXPD];
	enum battery_state	battery[CHANNELS_PERXPD];
	ushort			nobattery_debounce[CHANNELS_PERXPD];
	enum polarity_state	polarity[CHANNELS_PERXPD];
	ushort			polarity_debounce[CHANNELS_PERXPD];
	xpp_line_t		maybe_power_denial;
	ushort			power_denial_debounce[CHANNELS_PERXPD];
	ushort			power_denial_delay[CHANNELS_PERXPD];
	ushort			power_denial_safezone[CHANNELS_PERXPD];
	xpp_line_t		ledstate[NUM_LEDS];	/* 0 - OFF, 1 - ON */
	xpp_line_t		ledcontrol[NUM_LEDS];	/* 0 - OFF, 1 - ON */
	int			led_counter[NUM_LEDS][CHANNELS_PERXPD];
	atomic_t		ring_debounce[CHANNELS_PERXPD];
#ifdef	WITH_METERING
	uint			metering_count[CHANNELS_PERXPD];
	xpp_line_t		metering_tone_state;
#endif
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

/*---------------- FXO: Static functions ----------------------------------*/

static void reset_battery_readings(xpd_t *xpd, lineno_t pos)
{
	struct FXO_priv_data	*priv = xpd->priv;

	priv->nobattery_debounce[pos] = 0;
	priv->power_denial_debounce[pos] = 0;
	priv->power_denial_delay[pos] = 0;
	BIT_CLR(priv->maybe_power_denial, pos);
}

/*
 * LED control is done via DAA register 0x20
 */
static int do_led(xpd_t *xpd, lineno_t chan, byte which, bool on)
{
	int			ret = 0;
	struct FXO_priv_data	*priv;
	xbus_t			*xbus;

	BUG_ON(!xpd);
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
	ret = DAA_DIRECT_REQUEST(xbus, xpd, chan, DAA_WRITE, 0x20, on);
out:
	return ret;
}

static void handle_fxo_leds(xpd_t *xpd)
{
	int			i;
	unsigned long		flags;
	const enum fxo_leds	color = LED_GREEN;
	unsigned int		timer_count;
	struct FXO_priv_data	*priv;

	BUG_ON(!xpd);
	spin_lock_irqsave(&xpd->lock, flags);
	priv = xpd->priv;
	timer_count = xpd->timer_count;
	for_each_line(xpd, i) {
		if(IS_SET(xpd->digital_outputs, i) || IS_SET(xpd->digital_inputs, i))
			continue;
		if(xpd->blink_mode || IS_BLINKING(priv,i,color)) {
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
		} else if(IS_SET(priv->ledcontrol[color], i) && !IS_SET(priv->ledstate[color], i)) {
			do_led(xpd, i, color, 1);
		} else if(!IS_SET(priv->ledcontrol[color], i) && IS_SET(priv->ledstate[color], i)) {
			do_led(xpd, i, color, 0);
		}
	}
	spin_unlock_irqrestore(&xpd->lock, flags);
}

static void update_zap_ring(xpd_t *xpd, int pos, bool on)
{
	zt_rxsig_t	rxsig;

	BUG_ON(!xpd);
	if(on) {
		if(caller_id_style == CID_STYLE_BELL) {
			LINE_DBG(SIGNAL, xpd, pos, "Caller-ID PCM: off\n");
			BIT_CLR(xpd->cid_on, pos);
		}
		rxsig = ZT_RXSIG_RING;
	} else {
		if(caller_id_style == CID_STYLE_BELL) {
			LINE_DBG(SIGNAL, xpd, pos, "Caller-ID PCM: on\n");
			BIT_SET(xpd->cid_on, pos);
		}
		rxsig = ZT_RXSIG_OFFHOOK;
	}
	pcm_recompute(xpd, 0);
	/*
	 * We should not spinlock before calling zt_hooksig() as
	 * it may call back into our xpp_hooksig() and cause
	 * a nested spinlock scenario
	 */
	if(SPAN_REGISTERED(xpd))
		zt_hooksig(&xpd->chans[pos], rxsig);
}

static void mark_ring(xpd_t *xpd, lineno_t pos, bool on, bool update_zap)
{
	struct FXO_priv_data	*priv;

	priv = xpd->priv;
	BUG_ON(!priv);
	atomic_set(&priv->ring_debounce[pos], 0);	/* Stop debouncing */
	/*
	 * We don't want to check battery during ringing
	 * due to voltage fluctuations.
	 */
	reset_battery_readings(xpd, pos);
	if(on && !xpd->ringing[pos]) {
		LINE_DBG(SIGNAL, xpd, pos, "START\n");
		xpd->ringing[pos] = 1;
		MARK_BLINK(priv, pos, LED_GREEN, LED_BLINK_RING);
		if(update_zap)
			update_zap_ring(xpd, pos, on);
	} else if(!on && xpd->ringing[pos]) {
		LINE_DBG(SIGNAL, xpd, pos, "STOP\n");
		xpd->ringing[pos] = 0;
		if(IS_BLINKING(priv, pos, LED_GREEN))
			MARK_BLINK(priv, pos, LED_GREEN, 0);
		if(update_zap)
			update_zap_ring(xpd, pos, on);
	}
}

static int do_sethook(xpd_t *xpd, int pos, bool to_offhook)
{
	unsigned long		flags;
	xbus_t			*xbus;
	struct FXO_priv_data	*priv;
	int			ret = 0;
	byte			value;

	BUG_ON(!xpd);
	BUG_ON(xpd->direction == TO_PHONE);		// We can SETHOOK state only on PSTN
	xbus = xpd->xbus;
	priv = xpd->priv;
	BUG_ON(!priv);
	if(priv->battery[pos] != BATTERY_ON && to_offhook) {
		LINE_NOTICE(xpd, pos, "Cannot take offhook while battery is off!\n");
		return -EINVAL;
	}
	spin_lock_irqsave(&xpd->lock, flags);
	mark_ring(xpd, pos, 0, 0);				// No more rings
	value = (to_offhook) ? 0x09 : 0x08;	/* Bit 3 is for CID */
	LINE_DBG(SIGNAL, xpd, pos, "SETHOOK: value=0x%02X %s\n", value, (to_offhook)?"OFFHOOK":"ONHOOK");
	if(to_offhook)
		MARK_ON(priv, pos, LED_GREEN);
	else
		MARK_OFF(priv, pos, LED_GREEN);
	ret = DAA_DIRECT_REQUEST(xbus, xpd, pos, DAA_WRITE, DAA_REG_RING, value);
	if(to_offhook) {
		BIT_SET(xpd->offhook, pos);
	} else {
		BIT_CLR(xpd->offhook, pos);
	}
	if(caller_id_style != CID_STYLE_PASS_ALWAYS) {
		LINE_DBG(SIGNAL, xpd, pos, "Caller-ID PCM: off\n");
		BIT_CLR(xpd->cid_on, pos);
	}
#ifdef	WITH_METERING
	priv->metering_count[pos] = 0;
	priv->metering_tone_state = 0L;
	DAA_DIRECT_REQUEST(xbus, xpd, pos, DAA_WRITE, DAA_REG_METERING, 0x2D);
#endif
	reset_battery_readings(xpd, pos);	/* unstable during hook changes */
	priv->power_denial_safezone[pos] = (to_offhook) ? POWER_DENIAL_SAFEZONE : 0;
	spin_unlock_irqrestore(&xpd->lock, flags);
	return ret;
}

/*---------------- FXO: Methods -------------------------------------------*/

static xpd_t *FXO_card_new(xbus_t *xbus, int unit, int subunit, const xproto_table_t *proto_table, byte subtype, byte revision)
{
	xpd_t		*xpd = NULL;
	int		channels;

	if(subtype == 2)
		channels = min(2, CHANNELS_PERXPD);
	else
		channels = min(8, CHANNELS_PERXPD);
	xpd = xpd_alloc(sizeof(struct FXO_priv_data), proto_table, channels);
	if(!xpd)
		return NULL;
	xpd->direction = TO_PSTN;
	xpd->revision = revision;
	xpd->type_name = proto_table->name;
	return xpd;
}

static void clean_proc(xbus_t *xbus, xpd_t *xpd)
{
	struct FXO_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	XPD_DBG(PROC, xpd, "\n");
#ifdef	CONFIG_PROC_FS
	if(priv->regfile) {
		XPD_DBG(PROC, xpd, "Removing xpd DAA file\n");
		remove_proc_entry(PROC_REGISTER_FNAME, xpd->proc_xpd_dir);
		priv->regfile->data = NULL;
	}
#ifdef	WITH_METERING
	if(priv->meteringfile) {
		XPD_DBG(PROC, xpd, "Removing xpd metering tone file\n");
		priv->meteringfile->data = NULL;
		remove_proc_entry(PROC_METERING_FNAME, xpd->proc_xpd_dir);
		priv->meteringfile = NULL;
	}
#endif
	if(priv->fxo_info) {
		XPD_DBG(PROC, xpd, "Removing xpd FXO_INFO file\n");
		remove_proc_entry(PROC_FXO_INFO_FNAME, xpd->proc_xpd_dir);
		priv->fxo_info = NULL;
	}
#endif
}

static int FXO_card_init(xbus_t *xbus, xpd_t *xpd)
{
	struct FXO_priv_data	*priv;
	int			ret = 0;
	int			i;

	BUG_ON(!xpd);
	priv = xpd->priv;
#ifdef	CONFIG_PROC_FS
	XPD_DBG(PROC, xpd, "Creating FXO_INFO file\n");
	priv->fxo_info = create_proc_read_entry(PROC_FXO_INFO_FNAME, 0444, xpd->proc_xpd_dir, proc_fxo_info_read, xpd);
	if(!priv->fxo_info) {
		XPD_ERR(xpd, "Failed to create proc file '%s'\n", PROC_FXO_INFO_FNAME);
		ret = -ENOENT;
		goto err;
	}
	priv->fxo_info->owner = THIS_MODULE;
#ifdef	WITH_METERING
	XPD_DBG(PROC, xpd, "Creating Metering tone file\n");
	priv->meteringfile = create_proc_read_entry(PROC_METERING_FNAME, 0444, xpd->proc_xpd_dir,
			proc_xpd_metering_read, xpd);
	if(!priv->meteringfile) {
		XPD_ERR(xpd, "Failed to create proc file '%s'\n", PROC_METERING_FNAME);
		ret = -ENOENT;
		goto err;
	}
	priv->meteringfile->owner = THIS_MODULE;
#endif
	XPD_DBG(PROC, xpd, "Creating DAAs file\n");
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
	ret = run_initialize_registers(xpd);
	if(ret < 0)
		goto err;
	// Hanghup all lines
	for_each_line(xpd, i) {
		do_sethook(xpd, i, 0);
		priv->polarity[i] = POL_UNKNOWN;	/* will be updated on next battery sample */
		priv->battery[i] = BATTERY_UNKNOWN;	/* will be updated on next battery sample */
	}
	XPD_DBG(GENERAL, xpd, "done\n");
	for_each_line(xpd, i) {
		do_led(xpd, i, LED_GREEN, 0);
	}
	for_each_line(xpd, i) {
		do_led(xpd, i, LED_GREEN, 1);
		msleep(50);
	}
	for_each_line(xpd, i) {
		do_led(xpd, i, LED_GREEN, 0);
		msleep(50);
	}
	pcm_recompute(xpd, 0);
	return 0;
err:
	clean_proc(xbus, xpd);
	XPD_ERR(xpd, "Failed initializing registers (%d)\n", ret);
	return ret;
}

static int FXO_card_remove(xbus_t *xbus, xpd_t *xpd)
{
	struct FXO_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	XPD_DBG(GENERAL, xpd, "\n");
	clean_proc(xbus, xpd);
	return 0;
}

static int FXO_card_zaptel_preregistration(xpd_t *xpd, bool on)
{
	xbus_t			*xbus;
	struct FXO_priv_data	*priv;
	int			i;

	BUG_ON(!xpd);
	xbus = xpd->xbus;
	BUG_ON(!xbus);
	priv = xpd->priv;
	BUG_ON(!priv);
	XPD_DBG(GENERAL, xpd, "%s\n", (on)?"ON":"OFF");
#ifdef ZT_SPANSTAT_V2 
	xpd->span.spantype = "FXO";
#endif 
	for_each_line(xpd, i) {
		struct zt_chan	*cur_chan = &xpd->chans[i];

		XPD_DBG(GENERAL, xpd, "setting FXO channel %d\n", i);
		snprintf(cur_chan->name, MAX_CHANNAME, "XPP_FXO/%02d/%1d%1d/%d",
			xbus->num, xpd->addr.unit, xpd->addr.subunit, i);
		cur_chan->chanpos = i + 1;
		cur_chan->pvt = xpd;
		cur_chan->sigcap = FXO_DEFAULT_SIGCAP;
	}
	for_each_line(xpd, i) {
		MARK_ON(priv, i, LED_GREEN);
		msleep(4);
	}
	return 0;
}

static int FXO_card_zaptel_postregistration(xpd_t *xpd, bool on)
{
	xbus_t			*xbus;
	struct FXO_priv_data	*priv;
	int			i;

	BUG_ON(!xpd);
	xbus = xpd->xbus;
	BUG_ON(!xbus);
	priv = xpd->priv;
	BUG_ON(!priv);
	XPD_DBG(GENERAL, xpd, "%s\n", (on)?"ON":"OFF");
	for_each_line(xpd, i) {
		zap_report_battery(xpd, i);
		MARK_OFF(priv, i, LED_GREEN);
		msleep(2);
		// MARK_OFF(priv, i, LED_RED);
		msleep(2);
	}
	return 0;
}

static int FXO_card_hooksig(xbus_t *xbus, xpd_t *xpd, int pos, zt_txsig_t txsig)
{
	struct FXO_priv_data	*priv;
	int			ret = 0;

	priv = xpd->priv;
	BUG_ON(!priv);
	LINE_DBG(SIGNAL, xpd, pos, "%s\n", txsig2str(txsig));
	BUG_ON(xpd->direction != TO_PSTN);
	/* XXX Enable hooksig for FXO XXX */
	switch(txsig) {
		case ZT_TXSIG_START:
			break;
		case ZT_TXSIG_OFFHOOK:
			ret = do_sethook(xpd, pos, 1);
			break;
		case ZT_TXSIG_ONHOOK:
			ret = do_sethook(xpd, pos, 0);
			break;
		default:
			XPD_NOTICE(xpd, "Can't set tx state to %s (%d)\n",
				txsig2str(txsig), txsig);
			return -EINVAL;
	}
	pcm_recompute(xpd, 0);
	return ret;
}

static void zap_report_battery(xpd_t *xpd, lineno_t chan)
{
	struct FXO_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	if(SPAN_REGISTERED(xpd)) {
		switch(priv->battery[chan]) {
			case BATTERY_UNKNOWN:
				/* no-op */
				break;
			case BATTERY_OFF:
				LINE_DBG(SIGNAL, xpd, chan, "Send ZT_ALARM_RED\n");
				zt_alarm_channel(&xpd->chans[chan], ZT_ALARM_RED);
				break;
			case BATTERY_ON:
				LINE_DBG(SIGNAL, xpd, chan, "Send ZT_ALARM_NONE\n");
				zt_alarm_channel(&xpd->chans[chan], ZT_ALARM_NONE);
				break;
		}
	}
}

static int FXO_card_open(xpd_t *xpd, lineno_t chan)
{
	struct FXO_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	return 0;
}

static void poll_battery(xbus_t *xbus, xpd_t *xpd)
{
	int	i;

	for_each_line(xpd, i) {
		DAA_DIRECT_REQUEST(xbus, xpd, i, DAA_READ, DAA_REG_VBAT, 0);
	}
}

static void poll_current(xbus_t *xbus, xpd_t *xpd)
{
	int	i;

	for_each_line(xpd, i) {
		if (IS_SET(xpd->offhook, i))
			DAA_DIRECT_REQUEST(xbus, xpd, i, DAA_READ, DAA_REG_CURRENT, 0);
	}
}

#ifdef	WITH_METERING
static void poll_metering(xbus_t *xbus, xpd_t *xpd)
{
	int	i;

	for_each_line(xpd, i) {
		if (IS_SET(xpd->offhook, i))
			DAA_DIRECT_REQUEST(xbus, xpd, i, DAA_READ, DAA_REG_METERING, 0);
	}
}
#endif

static void handle_fxo_ring(xpd_t *xpd)
{
	struct FXO_priv_data	*priv;
	int			i;

	priv = xpd->priv;
	for_each_line(xpd, i) {
		if(atomic_read(&priv->ring_debounce[i]) > 0) {
			/* Maybe start ring */
			if(atomic_dec_and_test(&priv->ring_debounce[i]))
				mark_ring(xpd, i, 1, 1);
		} else if (atomic_read(&priv->ring_debounce[i]) < 0) {
			/* Maybe stop ring */
			if(atomic_inc_and_test(&priv->ring_debounce[i]))
				mark_ring(xpd, i, 0, 1);
		}
	}
}

static void handle_fxo_power_denial(xpd_t *xpd)
{
	struct FXO_priv_data	*priv;
	int			i;

	priv = xpd->priv;
	for_each_line(xpd, i) {
		if(priv->power_denial_safezone[i] > 0)
			priv->power_denial_safezone[i]--;
		if(IS_SET(priv->maybe_power_denial, i) && !xpd->ringing[i] && IS_SET(xpd->offhook, i)) {
			/*
			 * Ring detection by the firmware takes some time.
			 * Therefore we delay our decision until we are
			 * sure that no ring has started during this time.
			 */
			priv->power_denial_delay[i]++;
			if (priv->power_denial_delay[i] >= POWER_DENIAL_DELAY) {
				LINE_DBG(SIGNAL, xpd, i, "Power Denial Hangup\n");
				priv->power_denial_delay[i] = 0;
				BIT_CLR(priv->maybe_power_denial, i);
				do_sethook(xpd, i, 0);
				update_line_status(xpd, i, 0);
				pcm_recompute(xpd, 0);
			}
		} else {
			priv->power_denial_delay[i] = 0;
			BIT_CLR(priv->maybe_power_denial, i);
		}
	}
}

static int FXO_card_tick(xbus_t *xbus, xpd_t *xpd)
{
	struct FXO_priv_data	*priv;

	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	if(poll_battery_interval != 0 && (priv->poll_counter % poll_battery_interval) == 0)
		poll_battery(xbus, xpd);
	if(poll_power_denial_interval != 0 && (priv->poll_counter % poll_power_denial_interval) == 0)
		poll_current(xbus, xpd);
#ifdef	WITH_METERING
	if(poll_metering_interval != 0 && (priv->poll_counter % poll_metering_interval) == 0)
		poll_metering(xbus, xpd);
#endif
	handle_fxo_leds(xpd);
	handle_fxo_ring(xpd);
	handle_fxo_power_denial(xpd);
	priv->poll_counter++;
	return 0;
}

/* FIXME: based on data from from wctdm.h */
#include <wctdm.h>
/*
 * The first register is the ACIM, the other are coefficient registers.
 * We define the array size explicitly to track possible inconsistencies
 * if the struct is modified.
 */
static const char echotune_regs[sizeof(struct wctdm_echo_coefs)] = {30, 45, 46, 47, 48, 49, 50, 51, 52};

static int FXO_card_ioctl(xpd_t *xpd, int pos, unsigned int cmd, unsigned long arg)
{
	int 			i,ret;
	unsigned char		echotune_data[ARRAY_SIZE(echotune_regs)];

	BUG_ON(!xpd);
	if(!TRANSPORT_RUNNING(xpd->xbus))
		return -ENODEV;
	switch (cmd) {
		case WCTDM_SET_ECHOTUNE:
			XPD_DBG(GENERAL, xpd, "-- Setting echo registers: \n");
			/* first off: check if this span is fxs. If not: -EINVALID */
			if (copy_from_user(&echotune_data, (void __user *)arg, sizeof(echotune_data)))
				return -EFAULT;

			for (i = 0; i < ARRAY_SIZE(echotune_regs); i++) {
				XPD_DBG(REGS, xpd, "Reg=0x%02X, data=0x%02X\n", echotune_regs[i], echotune_data[i]);
				ret = DAA_DIRECT_REQUEST(xpd->xbus, xpd, pos, DAA_WRITE, echotune_regs[i], echotune_data[i]);
				if (ret < 0) {
					LINE_NOTICE(xpd, pos, "Couldn't write %0x02X to register %0x02X\n",
							echotune_data[i], echotune_regs[i]);
					return ret;
				}
				msleep(1);
			}

			XPD_DBG(GENERAL, xpd, "-- Set echo registers successfully\n");
			break;
		case ZT_TONEDETECT:
			/*
			 * Asterisk call all span types with this (FXS specific)
			 * call. Silently ignore it.
			 */
			LINE_DBG(GENERAL, xpd, pos,
				"ZT_TONEDETECT (FXO: NOTIMPLEMENTED)\n");
			return -ENOTTY;
		default:
			report_bad_ioctl(THIS_MODULE->name, xpd, pos, cmd);
			return -ENOTTY;
	}
	return 0;
}

/*---------------- FXO: HOST COMMANDS -------------------------------------*/

/* 0x0F */ HOSTCMD(FXO, XPD_STATE, bool on)
{
	int			ret = 0;
	struct FXO_priv_data	*priv;

	BUG_ON(!xbus);
	BUG_ON(!xpd);
	priv = xpd->priv;
	BUG_ON(!priv);
	XPD_DBG(GENERAL, xpd, "%s\n", (on) ? "on" : "off");
	return ret;
}

/* 0x0F */ HOSTCMD(FXO, RING, lineno_t chan, bool on)
{
	BUG_ON(!xbus);
	BUG_ON(!xpd);
	LINE_DBG(SIGNAL, xpd, chan, "%s\n", (on) ? "on" : "off");
	return DAA_DIRECT_REQUEST(xbus, xpd, chan, DAA_WRITE, 0x40, (on)?0x04:0x01);
}

/* 0x0F */ HOSTCMD(FXO, RELAY_OUT, byte which, bool on)
{
	return -ENOSYS;
}

/*---------------- FXO: Astribank Reply Handlers --------------------------*/

HANDLER_DEF(FXO, SIG_CHANGED)
{
	xpp_line_t	sig_status = RPACKET_FIELD(pack, FXO, SIG_CHANGED, sig_status);
	xpp_line_t	sig_toggles = RPACKET_FIELD(pack, FXO, SIG_CHANGED, sig_toggles);
	unsigned long	flags;
	int		i;
	struct FXO_priv_data	*priv;

	if(!xpd) {
		notify_bad_xpd(__FUNCTION__, xbus, XPACKET_ADDR(pack), cmd->name);
		return -EPROTO;
	}
	priv = xpd->priv;
	BUG_ON(!priv);
	XPD_DBG(SIGNAL, xpd, "(PSTN) sig_toggles=0x%04X sig_status=0x%04X\n", sig_toggles, sig_status);
	spin_lock_irqsave(&xpd->lock, flags);
	for_each_line(xpd, i) {
		int	debounce;

		if(IS_SET(sig_toggles, i)) {
			if(priv->battery[i] == BATTERY_OFF) {
				/*
				 * With poll_battery_interval==0 we cannot have BATTERY_OFF
				 * so we won't get here
				 */
				LINE_NOTICE(xpd, i, "SIG_CHANGED while battery is off. Ignored.\n");
				continue;
			}
			/* First report false ring alarms */
			debounce = atomic_read(&priv->ring_debounce[i]);
			if(debounce)
				LINE_NOTICE(xpd, i, "debounced false ring (only %d ticks)\n", debounce);
			/*
			 * Now set a new ring alarm.
			 * It will be checked in handle_fxo_ring()
			 */
			debounce = (IS_SET(sig_status, i)) ? ring_debounce : -ring_debounce;
			atomic_set(&priv->ring_debounce[i], debounce);
		}
	}
	spin_unlock_irqrestore(&xpd->lock, flags);
	return 0;
}

#ifndef ZT_GET_PARAMS_V1
#define zt_alarm_channel(a,b) zt_qevent_lock(a,( (b)==ZT_ALARM_NONE )? \
	ZT_EVENT_NOALARM : ZT_EVENT_ALARM)
#endif
static void update_battery_voltage(xpd_t *xpd, byte data_low, lineno_t chipsel)
{
	struct FXO_priv_data	*priv;
	enum polarity_state	pol;
	int			msec;

	priv = xpd->priv;
	BUG_ON(!priv);
	priv->battery_voltage[chipsel] = data_low;
	if(xpd->ringing[chipsel])
		goto ignore_reading;	/* ring voltage create false alarms */
	if(abs((signed char)data_low) < BAT_THRESHOLD) {
		/*
		 * Check for battery voltage fluctuations
		 */
		if(priv->battery[chipsel] != BATTERY_OFF) {
			int	milliseconds;

			milliseconds = priv->nobattery_debounce[chipsel]++ *
				poll_battery_interval;
			if(milliseconds > BAT_DEBOUNCE) {
				LINE_DBG(SIGNAL, xpd, chipsel, "BATTERY OFF voltage=%d\n", data_low);
				priv->battery[chipsel] = BATTERY_OFF;
				if(SPAN_REGISTERED(xpd))
					zap_report_battery(xpd, chipsel);
				priv->polarity[chipsel] = POL_UNKNOWN;	/* What's the polarity ? */
				/*
				 * Stop further processing for now
				 */
				goto ignore_reading;
			}

		}
	} else {
		priv->nobattery_debounce[chipsel] = 0;
		if(priv->battery[chipsel] != BATTERY_ON) {
			LINE_DBG(SIGNAL, xpd, chipsel, "BATTERY ON voltage=%d\n", data_low);
			priv->battery[chipsel] = BATTERY_ON;
			if(SPAN_REGISTERED(xpd))
				zap_report_battery(xpd, chipsel);
		}
	}
	if(priv->battery[chipsel] != BATTERY_ON) {
		priv->polarity[chipsel] = POL_UNKNOWN;	/* What's the polarity ? */
		return;
	}
	/*
	 * Handle reverse polarity
	 */
	if(data_low == 0)
		pol = POL_UNKNOWN;
	else if(IS_SET(data_low, 7))
		pol = POL_NEGATIVE;
	else
		pol = POL_POSITIVE;
	if(priv->polarity[chipsel] == pol) {
		/*
		 * Same polarity, reset debounce counter
		 */
		priv->polarity_debounce[chipsel] = 0;
		return;
	}
	/*
	 * Track polarity reversals and debounce spikes.
	 * Only reversals with long duration count.
	 */
	msec = priv->polarity_debounce[chipsel]++ * poll_battery_interval;
	if (msec >= POLREV_THRESHOLD) {
		priv->polarity_debounce[chipsel] = 0;
		if(pol != POL_UNKNOWN) {
			char	*polname = NULL;

			if(pol == POL_POSITIVE)
				polname = "Positive";
			else if(pol == POL_NEGATIVE)
				polname = "Negative";
			else
				BUG();
			LINE_DBG(SIGNAL, xpd, chipsel,
				"Polarity changed to %s\n", polname);
			/*
			 * Inform zaptel/Asterisk:
			 * 1. Maybe used for hangup detection during offhook
			 * 2. In some countries used to report caller-id during onhook
			 *    but before first ring.
			 */
			if(caller_id_style == CID_STYLE_ETSI_POLREV) {
				LINE_DBG(SIGNAL, xpd, chipsel, "Caller-ID PCM: on\n");
				BIT_SET(xpd->cid_on, chipsel);	/* will be cleared on ring/offhook */
			}
			if(SPAN_REGISTERED(xpd)) {
				LINE_DBG(SIGNAL, xpd, chipsel,
					"Send ZT_EVENT_POLARITY: %s\n", polname);
				zt_qevent_lock(&xpd->chans[chipsel], ZT_EVENT_POLARITY);
			}
		}
		priv->polarity[chipsel] = pol;
	}
	return;
ignore_reading:
	/*
	 * Reset debounce counters to prevent false alarms
	 */
	reset_battery_readings(xpd, chipsel);	/* unstable during hook changes */
}

static void update_battery_current(xpd_t *xpd, byte data_low, lineno_t chipsel)
{
	struct FXO_priv_data	*priv;

	priv = xpd->priv;
	BUG_ON(!priv);
	priv->battery_current[chipsel] = data_low;
	/*
	 * During ringing, current is not stable.
	 * During onhook there should not be current anyway.
	 */
	if(xpd->ringing[chipsel] || !IS_SET(xpd->offhook, chipsel))
		goto ignore_it;
	/*
	 * Power denial with no battery voltage is meaningless
	 */
	if(priv->battery[chipsel] != BATTERY_ON)
		goto ignore_it;
	/* Safe zone after offhook */
	if(priv->power_denial_safezone[chipsel] > 0)
		goto ignore_it;
	if(data_low < POWER_DENIAL_CURRENT) {
		/* Current dropped -- is it long enough (minimum ~80msec) */
		priv->power_denial_debounce[chipsel]++;
		if (priv->power_denial_debounce[chipsel] * poll_battery_interval >= POWER_DENIAL_TIME) {
			/*
			 * But maybe the FXS started to ring (and the firmware haven't
			 * detected it yet). This would cause false power denials.
			 * So we just flag it and schedule more ticks to wait.
			 * These ticks would elapse in handle_fxo_power_denial()
			 */
			LINE_DBG(SIGNAL, xpd, chipsel, "Possible Power Denial Hangup\n");
			priv->power_denial_debounce[chipsel] = 0;
			BIT_SET(priv->maybe_power_denial, chipsel);
		}
	}
	return;
ignore_it:
	BIT_CLR(priv->maybe_power_denial, chipsel);
	priv->power_denial_debounce[chipsel] = 0;
}

#ifdef	WITH_METERING
#define	BTD_BIT	BIT(0)

static void update_metering_state(xpd_t *xpd, byte data_low, lineno_t chipsel)
{
	struct FXO_priv_data	*priv;
	bool			metering_tone = data_low & BTD_BIT;
	bool			old_metering_tone;

	priv = xpd->priv;
	BUG_ON(!priv);
	old_metering_tone = IS_SET(priv->metering_tone_state, chipsel);
	LINE_DBG(SIGNAL, xpd, chipsel, "METERING: %s [dL=0x%X] (%d)\n",
		(metering_tone) ? "ON" : "OFF",
		data_low, priv->metering_count[chipsel]);
	if(metering_tone && !old_metering_tone) {
		/* Rising edge */
		priv->metering_count[chipsel]++;
		BIT_SET(priv->metering_tone_state, chipsel);
	} else if(!metering_tone && old_metering_tone)
		BIT_CLR(priv->metering_tone_state, chipsel);
	if(metering_tone) {
		/* Clear the BTD bit */
		data_low &= ~BTD_BIT;
		DAA_DIRECT_REQUEST(xpd->xbus, xpd, chipsel, DAA_WRITE, DAA_REG_METERING, data_low);
	}
}
#endif

static int FXO_card_register_reply(xbus_t *xbus, xpd_t *xpd, reg_cmd_t *info)
{
	struct FXO_priv_data	*priv;
	lineno_t		chipsel;

	priv = xpd->priv;
	BUG_ON(!priv);
	chipsel = REG_FIELD(info, chipsel);
	switch(REG_FIELD(info, regnum)) {
		case DAA_REG_VBAT:
			update_battery_voltage(xpd, REG_FIELD(info, data_low), chipsel);
			break;
		case DAA_REG_CURRENT:
			update_battery_current(xpd, REG_FIELD(info, data_low), chipsel);
			break;
#ifdef	WITH_METERING
		case DAA_REG_METERING:
			update_metering_state(xpd, REG_FIELD(info, data_low), chipsel);
			break;
#endif
	}
	LINE_DBG(REGS, xpd, chipsel, "%c reg_num=0x%X, dataL=0x%X dataH=0x%X\n",
			((info->bytes == 3)?'I':'D'),
			REG_FIELD(info, regnum),
			REG_FIELD(info, data_low),
			REG_FIELD(info, data_high));
	/* Update /proc info only if reply relate to the last slic read request */
	if(
			REG_FIELD(&xpd->requested_reply, regnum) == REG_FIELD(info, regnum) &&
			REG_FIELD(&xpd->requested_reply, do_subreg) == REG_FIELD(info, do_subreg) &&
			REG_FIELD(&xpd->requested_reply, subreg) == REG_FIELD(info, subreg)) {
		xpd->last_reply = *info;
	}
	return 0;
}


static xproto_table_t PROTO_TABLE(FXO) = {
	.owner = THIS_MODULE,
	.entries = {
		/*	Prototable	Card	Opcode		*/
		XENTRY(	FXO,		FXO,	SIG_CHANGED	),
	},
	.name = "FXO",
	.type = XPD_TYPE_FXO,
	.xops = {
		.card_new	= FXO_card_new,
		.card_init	= FXO_card_init,
		.card_remove	= FXO_card_remove,
		.card_zaptel_preregistration	= FXO_card_zaptel_preregistration,
		.card_zaptel_postregistration	= FXO_card_zaptel_postregistration,
		.card_hooksig	= FXO_card_hooksig,
		.card_tick	= FXO_card_tick,
		.card_pcm_fromspan	= generic_card_pcm_fromspan,
		.card_pcm_tospan	= generic_card_pcm_tospan,
		.card_ioctl	= FXO_card_ioctl,
		.card_open	= FXO_card_open,
		.card_register_reply	= FXO_card_register_reply,

		.RING		= XPROTO_CALLER(FXO, RING),
		.RELAY_OUT	= XPROTO_CALLER(FXO, RELAY_OUT),
		.XPD_STATE	= XPROTO_CALLER(FXO, XPD_STATE),
	},
	.packet_is_valid = fxo_packet_is_valid,
	.packet_dump = fxo_packet_dump,
};

static bool fxo_packet_is_valid(xpacket_t *pack)
{
	const xproto_entry_t	*xe;

	//DBG(GENERAL, "\n");
	xe = xproto_card_entry(&PROTO_TABLE(FXO), XPACKET_OP(pack));
	return xe != NULL;
}

static void fxo_packet_dump(const char *msg, xpacket_t *pack)
{
	DBG(GENERAL, "%s\n", msg);
}

/*------------------------- DAA Handling --------------------------*/

static int proc_fxo_info_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int			len = 0;
	unsigned long		flags;
	xpd_t			*xpd = data;
	struct FXO_priv_data	*priv;
	int			i;

	if(!xpd)
		return -ENODEV;
	spin_lock_irqsave(&xpd->lock, flags);
	priv = xpd->priv;
	BUG_ON(!priv);
	len += sprintf(page + len, "\t%-17s: ", "Channel");
	for_each_line(xpd, i) {
		if(!IS_SET(xpd->digital_outputs, i) && !IS_SET(xpd->digital_inputs, i))
			len += sprintf(page + len, "%4d ", i % 10);
	}
	len += sprintf(page + len, "\nLeds:");
	len += sprintf(page + len, "\n\t%-17s: ", "state");
	for_each_line(xpd, i) {
		if(!IS_SET(xpd->digital_outputs, i) && !IS_SET(xpd->digital_inputs, i))
			len += sprintf(page + len, "%4d ", IS_SET(priv->ledstate[LED_GREEN], i));
	}
	len += sprintf(page + len, "\n\t%-17s: ", "blinking");
	for_each_line(xpd, i) {
		if(!IS_SET(xpd->digital_outputs, i) && !IS_SET(xpd->digital_inputs, i))
			len += sprintf(page + len, "%4d ", IS_BLINKING(priv,i,LED_GREEN));
	}
	len += sprintf(page + len, "\nBattery-Data:");
	len += sprintf(page + len, "\n\t%-17s: ", "voltage");
	for_each_line(xpd, i) {
		len += sprintf(page + len, "%4d ", priv->battery_voltage[i]);
	}
	len += sprintf(page + len, "\n\t%-17s: ", "current");
	for_each_line(xpd, i) {
		len += sprintf(page + len, "%4d ", priv->battery_current[i]);
	}
	len += sprintf(page + len, "\nBattery:");
	len += sprintf(page + len, "\n\t%-17s: ", "on");
	for_each_line(xpd, i) {
		char	*bat;

		if(priv->battery[i] == BATTERY_ON)
			bat = "+";
		else if(priv->battery[i] == BATTERY_OFF)
			bat = "-";
		else
			bat = ".";
		len += sprintf(page + len, "%4s ", bat);
	}
	len += sprintf(page + len, "\n\t%-17s: ", "debounce");
	for_each_line(xpd, i) {
		len += sprintf(page + len, "%4d ", priv->nobattery_debounce[i]);
	}
	len += sprintf(page + len, "\nPolarity-Reverse:");
	len += sprintf(page + len, "\n\t%-17s: ", "polarity");
	for_each_line(xpd, i) {
		char	*polname;

		if(priv->polarity[i] == POL_POSITIVE)
			polname = "+";
		else if(priv->polarity[i] == POL_NEGATIVE)
			polname = "-";
		else
			polname = ".";
		len += sprintf(page + len, "%4s ", polname);
	}
	len += sprintf(page + len, "\n\t%-17s: ", "debounce");
	for_each_line(xpd, i) {
		len += sprintf(page + len, "%4d ", priv->polarity_debounce[i]);
	}
	len += sprintf(page + len, "\nPower-Denial:");
	len += sprintf(page + len, "\n\t%-17s: ", "maybe");
	for_each_line(xpd, i) {
		len += sprintf(page + len, "%4d ", IS_SET(priv->maybe_power_denial, i));
	}
	len += sprintf(page + len, "\n\t%-17s: ", "debounce");
	for_each_line(xpd, i) {
		len += sprintf(page + len, "%4d ", priv->power_denial_debounce[i]);
	}
	len += sprintf(page + len, "\n\t%-17s: ", "safezone");
	for_each_line(xpd, i) {
		len += sprintf(page + len, "%4d ", priv->power_denial_safezone[i]);
	}
	len += sprintf(page + len, "\n\t%-17s: ", "delay");
	for_each_line(xpd, i) {
		len += sprintf(page + len, "%4d ", priv->power_denial_delay[i]);
	}
#ifdef	WITH_METERING
	len += sprintf(page + len, "\nMetering:");
	len += sprintf(page + len, "\n\t%-17s: ", "count");
	for_each_line(xpd, i) {
		len += sprintf(page + len, "%4d ", priv->metering_count[i]);
	}
#endif
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
	unsigned		data_low = 0;
	char			op;		/* [W]rite, [R]ead */
	char			reg_type;	/* [D]irect */
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
	elements = sscanf(cmdline, "%d %c%c %x %x",
			&chipsel,
			&op, &reg_type, &reg_num,
			&data_low);
	XPD_DBG(PROC, xpd, "'%s': %d %c%c %02X %02X\n", cmdline, chipsel, op, reg_type, reg_num, data_low);
	if(elements < 4) {	// At least: chipsel, op, reg_type, reg_num
		ERR("Not enough arguments: (%d args) '%s'\n", elements, cmdline);
		goto out;
	}
	if(!VALID_CHIPSEL(chipsel)) {
		ERR("Bad chipsel number: %d\n", chipsel);
		goto out;
	}
	REG_FIELD(&regcmd, chipsel) = chipsel;
	REG_FIELD(&regcmd, do_subreg) = 0;
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
		case 'D':
			REG_FIELD(&regcmd, regnum) = reg_num;
			REG_FIELD(&regcmd, subreg) = 0;
			break;
		default:
			ERR("Unkown register type '%c'\n", reg_type);
			goto out;
	}
	if(
			(op == 'W' && reg_type == 'D' && elements != 5) ||
			(op == 'R' && reg_type == 'D' && elements != 4)
	  ) {
		ERR("%s: '%s' (%d elements): %d %c%c %02X %02X\n", __FUNCTION__,
				cmdline, elements,
				chipsel, op, reg_type, reg_num, data_low);
		goto out;
	}
	regcmd.bytes = sizeof(regcmd) - 1;
	REG_FIELD(&regcmd, data_low) = data_low;
	REG_FIELD(&regcmd, data_high) = 0;
	REG_FIELD(&regcmd, read_request) = writing;
	xpd->requested_reply = regcmd;
	if(print_dbg)
		dump_reg_cmd("FXO", &regcmd, 1);
	ret = DAA_DIRECT_REQUEST(xpd->xbus, xpd, REG_FIELD(&regcmd, chipsel), writing, REG_FIELD(&regcmd, regnum), REG_FIELD(&regcmd, data_low));
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

	if(!xpd)
		return -ENODEV;
	spin_lock_irqsave(&xpd->lock, flags);
	info = &xpd->last_reply;
	regnum = REG_FIELD(info, regnum);
	len += sprintf(page + len, "# Writing bad data into this file may damage your hardware!\n");
	len += sprintf(page + len, "# Consult firmware docs first\n");
	len += sprintf(page + len, "#\n");
	len += sprintf(page + len, "#CH\tD/I\tReg.\tDL\n");
	len += sprintf(page + len, "%2d\tRD\t%02X\t%02X\n",
			REG_FIELD(info, chipsel),
			regnum, REG_FIELD(info, data_low));
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
static int proc_xpd_metering_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int			len = 0;
	unsigned long		flags;
	xpd_t			*xpd = data;
	struct FXO_priv_data	*priv;
	int			i;

	if(!xpd)
		return -ENODEV;
	priv = xpd->priv;
	BUG_ON(!priv);
	spin_lock_irqsave(&xpd->lock, flags);
	len += sprintf(page + len, "# Chan\tMeter (since last read)\n");
	for_each_line(xpd, i) {
		len += sprintf(page + len, "%d\t%d\n",
			i, priv->metering_count[i]);
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
	/* Zero meters */
	for_each_line(xpd, i)
		priv->metering_count[i] = 0;
	return len;
}
#endif

int __init card_fxo_startup(void)
{
	if(ring_debounce <= 0) {
		ERR("ring_debounce=%d. Must be positive number of ticks\n", ring_debounce);
		return -EINVAL;
	}
	INFO("revision %s\n", XPP_VERSION);
#ifdef	WITH_METERING
	INFO("FEATURE: WITH METERING Detection\n");
#else
	INFO("FEATURE: NO METERING Detection\n");
#endif
	xproto_register(&PROTO_TABLE(FXO));
	return 0;
}

void __exit card_fxo_cleanup(void)
{
	xproto_unregister(&PROTO_TABLE(FXO));
}

MODULE_DESCRIPTION("XPP FXO Card Driver");
MODULE_AUTHOR("Oron Peled <oron@actcom.co.il>");
MODULE_LICENSE("GPL");
MODULE_VERSION(XPP_VERSION);
MODULE_ALIAS_XPD(XPD_TYPE_FXO);

module_init(card_fxo_startup);
module_exit(card_fxo_cleanup);
