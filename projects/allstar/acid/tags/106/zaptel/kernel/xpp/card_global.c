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

#include <linux/module.h>
#include "xdefs.h"
#include "xpd.h"
#include "xpp_zap.h"
#include "xproto.h"
#include "zap_debug.h"
#include "xbus-core.h"
#include "parport_debug.h"

static const char rcsid[] = "$Id: card_global.c 3957 2008-03-07 00:45:53Z tzafrir $";

DEF_PARM(charp,initdir, "/usr/share/zaptel", 0644, "The directory of card initialization scripts");

extern	int print_dbg;

/*---------------- GLOBAL Protocol Commands -------------------------------*/

static bool global_packet_is_valid(xpacket_t *pack);
static void global_packet_dump(const char *msg, xpacket_t *pack);

/*---------------- GLOBAL: HOST COMMANDS ----------------------------------*/

/* 0x04 */ HOSTCMD(GLOBAL, DESC_REQ, int xpd_num)
{
	int		ret = 0;
	xframe_t	*xframe;
	xpacket_t	*pack;

	if(!xbus) {
		DBG(GENERAL, "NO XBUS\n");
		return -EINVAL;
	}
	XFRAME_NEW_CMD(xframe, pack, xbus, GLOBAL, DESC_REQ, xpd_num);
	XBUS_DBG(GENERAL, xbus, "to %1d%1d\n", XBUS_UNIT(xpd_num), XBUS_SUBUNIT(xpd_num));
	ret = send_cmd_frame(xbus, xframe);
	XBUS_COUNTER(xbus, DESC_REQ)++;
	return ret;
}

int xpp_register_request(xbus_t *xbus, xpd_t *xpd,
	byte chipsel, bool writing, bool do_subreg, byte regnum, byte subreg, byte data_low, byte data_high)
{
	int		ret = 0;
	xframe_t	*xframe;
	xpacket_t	*pack;
	reg_cmd_t	*reg_cmd;

	if(!xbus) {
		DBG(REGS, "NO XBUS\n");
		return -EINVAL;
	}
	XFRAME_NEW_CMD(xframe, pack, xbus, GLOBAL, REGISTER_REQUEST, xpd->xbus_idx);
	LINE_DBG(REGS, xpd, chipsel, "%c%c R%02X S%02X %02X %02X\n",
			(writing)?'W':'R',
			(do_subreg)?'S':'D',
			regnum, subreg, data_low, data_high);
	reg_cmd = &RPACKET_FIELD(pack, GLOBAL, REGISTER_REQUEST, reg_cmd);
	reg_cmd->bytes = sizeof(*reg_cmd) - 1;	// do not count the 'bytes' field
	reg_cmd->eoframe = 0;
	reg_cmd->multibyte = 0;
	REG_FIELD(reg_cmd, chipsel) = chipsel;
	REG_FIELD(reg_cmd, reserved) = 0;	/* force reserved bits to 0 */
	REG_FIELD(reg_cmd, read_request) = (writing) ? 0 : 1;
	REG_FIELD(reg_cmd, do_subreg) = do_subreg;
	REG_FIELD(reg_cmd, regnum) = regnum;
	REG_FIELD(reg_cmd, subreg) = subreg;
	REG_FIELD(reg_cmd, data_low) = data_low;
	REG_FIELD(reg_cmd, data_high) = data_high;
	ret = send_cmd_frame(xbus, xframe);
	return ret;
}

/*
 * The XPD parameter is totaly ignored by the driver and firmware as well.
 */
/* 0x19 */ HOSTCMD(GLOBAL, SYNC_SOURCE, enum sync_mode mode, int drift)
{
	xframe_t	*xframe;
	xpacket_t	*pack;
	const char	*mode_name;

	BUG_ON(!xbus);
	if((mode_name = sync_mode_name(mode)) == NULL) {
		XBUS_ERR(xbus, "SYNC_SOURCE: bad sync_mode=0x%X\n", mode);
		return -EINVAL;
	}
	XBUS_DBG(SYNC, xbus, "%s (0x%X), drift=%d\n", mode_name, mode, drift);
	XFRAME_NEW_CMD(xframe, pack, xbus, GLOBAL, SYNC_SOURCE, 0);
	RPACKET_FIELD(pack, GLOBAL, SYNC_SOURCE, sync_mode) = mode;
	RPACKET_FIELD(pack, GLOBAL, SYNC_SOURCE, drift) = drift;
	send_cmd_frame(xbus, xframe);
	return 0;
}

/* 0x23 */ HOSTCMD(GLOBAL, RESET_SYNC_COUNTERS)
{
	xframe_t	*xframe;
	xpacket_t	*pack;

	BUG_ON(!xbus);
	//XBUS_DBG(SYNC, xbus, "\n");
	XFRAME_NEW_CMD(xframe, pack, xbus, GLOBAL, RESET_SYNC_COUNTERS, 0);
	RPACKET_FIELD(pack, GLOBAL, RESET_SYNC_COUNTERS, mask) = 0x10;
	send_cmd_frame(xbus, xframe);
	return 0;
}

/*---------------- GLOBAL: Astribank Reply Handlers -----------------------*/

HANDLER_DEF(GLOBAL, NULL_REPLY)
{
	XBUS_DBG(GENERAL, xbus, "got len=%d\n", XPACKET_LEN(pack));
	return 0;
}

HANDLER_DEF(GLOBAL, DEV_DESC)
{
	struct card_desc_struct	*card_desc;

	BUG_ON(!xbus);
	if((card_desc = KZALLOC(sizeof(struct card_desc_struct), GFP_ATOMIC)) == NULL) {
		XBUS_ERR(xbus, "Card description allocation failed.\n");
		return -ENOMEM;
	}
	card_desc->magic = CARD_DESC_MAGIC;
	INIT_LIST_HEAD(&card_desc->card_list);
	card_desc->xbus = xbus;
	card_desc->type = RPACKET_FIELD(pack, GLOBAL, DEV_DESC, type);
	card_desc->subtype = RPACKET_FIELD(pack, GLOBAL, DEV_DESC, subtype);
	card_desc->rev = RPACKET_FIELD(pack, GLOBAL, DEV_DESC, rev);
	card_desc->xpd_addr = RPACKET_FIELD(pack, GLOBAL, DEV_DESC, head.addr);
	card_desc->line_status = RPACKET_FIELD(pack, GLOBAL, DEV_DESC, line_status);
	XBUS_DBG(GENERAL, xbus, "XPD=%d%d type=%d.%d rev=%d line_status=0x%04X\n",
			card_desc->xpd_addr.unit,
			card_desc->xpd_addr.subunit,
			card_desc->type,
			card_desc->subtype,
			card_desc->rev,
			card_desc->line_status);
	xbus_poller_notify(xbus, card_desc);
	return 0;
}

HANDLER_DEF(GLOBAL, REGISTER_REPLY)
{
	reg_cmd_t		*reg = &RPACKET_FIELD(pack, GLOBAL, REGISTER_REPLY, regcmd);

	if(!xpd) {
		static int	rate_limit;

		if((rate_limit++ % 1003) < 5) {
			XBUS_NOTICE(xbus,
				"%s: non-existing unit (%1d%1d) (rate_limit=%d)\n",
				__FUNCTION__,
				XPACKET_ADDR_UNIT(pack),
				XPACKET_ADDR_SUBUNIT(pack),
				rate_limit);
		}
		return -EPROTO;
	}
	return CALL_XMETHOD(card_register_reply, xbus, xpd, reg);
}

HANDLER_DEF(GLOBAL, SYNC_REPLY)
{
	byte		mode = RPACKET_FIELD(pack, GLOBAL, SYNC_REPLY, sync_mode);
	byte		drift = RPACKET_FIELD(pack, GLOBAL, SYNC_REPLY, drift);
	const char	*mode_name;

	BUG_ON(!xbus);
	if((mode_name = sync_mode_name(mode)) == NULL) {
		XBUS_ERR(xbus, "SYNC_REPLY: bad sync_mode=0x%X\n", mode);
		return -EINVAL;
	}
	XBUS_DBG(SYNC, xbus, "%s (0x%X), drift=%d\n", mode_name, mode, drift);
	//dump_packet("SYNC_REPLY", pack, print_dbg & DBG_SYNC);
	got_new_syncer(xbus, mode, drift);
	return 0;
}

#define	TMP_NAME_LEN	(XBUS_NAMELEN + XPD_NAMELEN + 5)

HANDLER_DEF(GLOBAL, ERROR_CODE)
{
	byte		errorcode = RPACKET_FIELD(pack, GLOBAL, ERROR_CODE, errorcode);
	reg_cmd_t	*bad_cmd;
	char		tmp_name[TMP_NAME_LEN];
	static long	rate_limit;

	BUG_ON(!xbus);
	if((rate_limit++ % 5003) > 200)
		return 0;
	if(!xpd) {
		snprintf(tmp_name, TMP_NAME_LEN, "%s(%1d%1d)", xbus->busname,
			XPACKET_ADDR_UNIT(pack), XPACKET_ADDR_SUBUNIT(pack));
	} else {
		snprintf(tmp_name, TMP_NAME_LEN, "%s/%s", xbus->busname, xpd->xpdname);
	}
	NOTICE("%s: FIRMWARE: %s CODE = 0x%X (rate_limit=%ld)\n",
			tmp_name, cmd->name, errorcode, rate_limit);
	switch(errorcode) {
		case 1:
			bad_cmd = &RPACKET_FIELD(pack, GLOBAL, ERROR_CODE, info.bad_spi_cmd);
			dump_packet("FIRMWARE: BAD_SPI_CMD", pack, 1);
			break;
		case 0xAB:
			dump_packet("FIRMWARE: BAD_PACKET_LEN", pack, 1);
			break;
		default:
			NOTICE("%s: FIRMWARE: %s UNKNOWN CODE = 0x%X\n", tmp_name, cmd->name, errorcode);
			dump_packet("PACKET", pack, 1);
	}
	/*
	 * FIXME: Should implement an error recovery plan
	 */
	return 0;
}


xproto_table_t PROTO_TABLE(GLOBAL) = {
	.entries = {
		/*	Prototable	Card	Opcode		*/
		XENTRY(	GLOBAL,		GLOBAL, NULL_REPLY	),
		XENTRY(	GLOBAL,		GLOBAL, DEV_DESC	),
		XENTRY(	GLOBAL,		GLOBAL,	SYNC_REPLY	),
		XENTRY(	GLOBAL,		GLOBAL, ERROR_CODE	),
		XENTRY(	GLOBAL,		GLOBAL, REGISTER_REPLY	),
	},
	.name = "GLOBAL",
	.packet_is_valid = global_packet_is_valid,
	.packet_dump = global_packet_dump,
};

static bool global_packet_is_valid(xpacket_t *pack)
{
	const xproto_entry_t	*xe;

	//DBG(GENERAL, "\n");
	xe = xproto_global_entry(XPACKET_OP(pack));
	return xe != NULL;
}

static void global_packet_dump(const char *msg, xpacket_t *pack)
{
	DBG(GENERAL, "%s\n", msg);
}

#define	MAX_ENV_STR	40
#define	MAX_PATH_STR	60

int run_initialize_registers(xpd_t *xpd)
{
	int	ret;
	xbus_t	*xbus;
	char	busstr[MAX_ENV_STR];
	char	xpdstr[MAX_ENV_STR];
	char	unitstr[MAX_ENV_STR];
	char	subunitstr[MAX_ENV_STR];
	char	typestr[MAX_ENV_STR];
	char	revstr[MAX_ENV_STR];
	char	connectorstr[MAX_ENV_STR];
	char	init_card[MAX_PATH_STR];
	char	*argv[] = {
		init_card,
		NULL
	};
	char	*envp[] = {
		busstr,
		xpdstr,
		unitstr,
		subunitstr,
		typestr,
		revstr,
		connectorstr,
		NULL
	};

	BUG_ON(!xpd);
	xbus = xpd->xbus;
	if(!initdir || !initdir[0]) {
		XPD_NOTICE(xpd, "Missing initdir parameter\n");
		return -EINVAL;
	}
	snprintf(busstr, MAX_ENV_STR, "XPD_BUS=%s", xbus->busname);
	snprintf(xpdstr, MAX_ENV_STR, "XPD_NAME=%s", xpd->xpdname);
	snprintf(unitstr, MAX_ENV_STR, "XPD_UNIT=%d", xpd->addr.unit);
	snprintf(subunitstr, MAX_ENV_STR, "XPD_SUBUNIT=%d", xpd->addr.subunit);
	snprintf(typestr, MAX_ENV_STR, "XPD_TYPE=%d", xpd->type);
	snprintf(revstr, MAX_ENV_STR, "XPD_REVISION=%d", xpd->revision);
	snprintf(connectorstr, MAX_ENV_STR, "XBUS_CONNECTOR=%s", xbus->location);
	if(snprintf(init_card, MAX_PATH_STR, "%s/init_card_%d_%d",
				initdir, xpd->type, xpd->revision) > MAX_PATH_STR) {
		XPD_NOTICE(xpd, "Cannot initialize. pathname is longer than %d characters.\n", MAX_PATH_STR);
		return -E2BIG;
	}
	if(!XBUS_GET(xbus)) {
		XBUS_ERR(xbus, "Skipped register initialization. XBUS is going down\n");
		return -ENODEV;
	}
	XPD_DBG(GENERAL, xpd, "running '%s' for type=%d revision=%d\n",
			init_card, xpd->type, xpd->revision);
	ret = call_usermodehelper(init_card, argv, envp, 1);
	/*
	 * Carefully report results
	 */
	if(ret == 0)
		XPD_DBG(GENERAL, xpd, "'%s' finished OK\n", init_card);
	else if(ret < 0) {
		XPD_ERR(xpd, "Failed running '%s' (errno %d)\n", init_card, ret);
	} else {
		byte	exitval = ((unsigned)ret >> 8) & 0xFF;
		byte	sigval = ret & 0xFF;

		if(!exitval) {
			XPD_ERR(xpd, "'%s' killed by signal %d\n", init_card, sigval);
		} else {
			XPD_ERR(xpd, "'%s' aborted with exitval %d\n", init_card, exitval);
		}
		ret = -EINVAL;
	}
	XBUS_PUT(xbus);
	return ret;
}

EXPORT_SYMBOL(sync_mode_name);
EXPORT_SYMBOL(run_initialize_registers);
EXPORT_SYMBOL(xpp_register_request);
