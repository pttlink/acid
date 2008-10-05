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

#include "xpd.h"
#include "xproto.h"
#include "xpp_zap.h"
#include "xbus-core.h"
#include "zap_debug.h"
#include <linux/module.h>
#include <linux/delay.h>

static const char rcsid[] = "$Id: xproto.c 3957 2008-03-07 00:45:53Z tzafrir $";

extern	int print_dbg;

static const xproto_table_t *xprotocol_tables[XPD_TYPE_NOMODULE];

#if MAX_UNIT*MAX_SUBUNIT > MAX_XPDS
#error MAX_XPDS is too small
#endif

bool valid_xpd_addr(const struct xpd_addr *addr)
{
	return ((addr->subunit & ~BITMASK(SUBUNIT_BITS)) == 0) && ((addr->unit & ~BITMASK(UNIT_BITS)) == 0);
}

/*---------------- General Protocol Management ----------------------------*/

const xproto_entry_t *xproto_card_entry(const xproto_table_t *table, byte opcode)
{
	const xproto_entry_t *xe;

	//DBG(GENERAL, "\n");
	xe = &table->entries[opcode];
	return (xe->handler != NULL) ? xe : NULL;
}

const xproto_entry_t *xproto_global_entry(byte opcode)
{
	const xproto_entry_t *xe;

	xe = xproto_card_entry(&PROTO_TABLE(GLOBAL), opcode);
	//DBG(GENERAL, "opcode=0x%X xe=%p\n", opcode, xe);
	return xe;
}

xproto_handler_t xproto_global_handler(byte opcode)
{
	return xproto_card_handler(&PROTO_TABLE(GLOBAL), opcode);
}

const xproto_table_t *xproto_table(xpd_type_t cardtype)
{
	if(cardtype >= XPD_TYPE_NOMODULE)
		return NULL;
	return xprotocol_tables[cardtype];
}

const xproto_table_t *xproto_get(xpd_type_t cardtype)
{
	const xproto_table_t *xtable;

	if(cardtype >= XPD_TYPE_NOMODULE)
		return NULL;
	xtable = xprotocol_tables[cardtype];
	if(!xtable) {	/* Try to load the relevant module */
		int ret = request_module(XPD_TYPE_PREFIX "%d", cardtype);
		if(ret != 0) {
			NOTICE("%s: Failed to load module for type=%d. exit status=%d.\n",
					__FUNCTION__, cardtype, ret);
			/* Drop through: we may be lucky... */
		}
		xtable = xprotocol_tables[cardtype];
	}
	if(xtable) {
		BUG_ON(!xtable->owner);
		DBG(GENERAL, "%s refcount was %d\n", xtable->name, module_refcount(xtable->owner));
		if(!try_module_get(xtable->owner)) {
			ERR("%s: try_module_get for %s failed.\n", __FUNCTION__, xtable->name);
			return NULL;
		}
	}
	return xtable;
}

void xproto_put(const xproto_table_t *xtable)
{
	BUG_ON(!xtable);
	DBG(GENERAL, "%s refcount was %d\n", xtable->name, module_refcount(xtable->owner));
	BUG_ON(module_refcount(xtable->owner) <= 0);
	module_put(xtable->owner);
}

xproto_handler_t xproto_card_handler(const xproto_table_t *table, byte opcode)
{
	const xproto_entry_t *xe;

	//DBG(GENERAL, "\n");
	xe = xproto_card_entry(table, opcode);
	return xe->handler;
}

void notify_bad_xpd(const char *funcname, xbus_t *xbus, const struct xpd_addr addr, const char *msg)
{
	XBUS_NOTICE(xbus, "%s: non-existing address (%1d%1d): %s\n",
			funcname, addr.unit, addr.subunit, msg);
}

int packet_process(xbus_t *xbus, xpacket_t *pack)
{
	byte			op;
	const xproto_entry_t	*xe;
	xproto_handler_t	handler;
	xproto_table_t		*table;
	xpd_t			*xpd;
	int			ret = -EPROTO;

	BUG_ON(!pack);
	if(!valid_xpd_addr(&XPACKET_ADDR(pack))) {
		if(printk_ratelimit()) {
			XBUS_NOTICE(xbus, "%s: from %d%d: bad address.\n",
					__FUNCTION__,
					XPACKET_ADDR_UNIT(pack), XPACKET_ADDR_SUBUNIT(pack));
			dump_packet("packet_process -- bad address", pack, print_dbg);
		}
		goto out;
	}
	op = XPACKET_OP(pack);
	xpd = xpd_byaddr(xbus, XPACKET_ADDR_UNIT(pack), XPACKET_ADDR_SUBUNIT(pack));
	/* XPD may be NULL (e.g: during bus polling */
	xe = xproto_global_entry(op);
	/*-------- Validations -----------*/
	if(!xe) {
		const xproto_table_t *xtable;
		
		if(!xpd) {
			if(printk_ratelimit()) {
				XBUS_NOTICE(xbus, "%s: from %d%d opcode=0x%02X: no such global command.\n",
						__FUNCTION__,
						XPACKET_ADDR_UNIT(pack), XPACKET_ADDR_SUBUNIT(pack), op);
				dump_packet("packet_process -- no such global command", pack, 1);
			}
			goto out;
		}
		xtable = xproto_table(xpd->type);
		if(!xtable) {
			if(printk_ratelimit())
				XPD_ERR(xpd, "%s: no protocol table (type=%d)\n",
					__FUNCTION__,
					xpd->type);
			goto out;
		}
		xe = xproto_card_entry(xtable, op);
		if(!xe) {
			if(printk_ratelimit()) {
				XPD_NOTICE(xpd, "%s: bad command (type=%d,opcode=0x%x)\n",
					__FUNCTION__,
					xpd->type, op);
				dump_packet("packet_process -- bad command", pack, 1);
			}
			goto out;
		}
	}
	table = xe->table;
	BUG_ON(!table);
	if(!table->packet_is_valid(pack)) {
		if(printk_ratelimit()) {
			ERR("xpp: %s: wrong size %d for opcode=0x%02X\n",
					__FUNCTION__, XPACKET_LEN(pack), op);
			dump_packet("packet_process -- wrong size", pack, print_dbg);
		}
		goto out;
	}
	ret = 0;	/* All well */
	handler = xe->handler;
	BUG_ON(!handler);
	XBUS_COUNTER(xbus, RX_BYTES) += XPACKET_LEN(pack);
	handler(xbus, xpd, xe, pack);
out:
	return ret;
}

static int xframe_receive_cmd(xbus_t *xbus, xframe_t *xframe)
{
	byte		*xframe_end;
	xpacket_t	*pack;
	byte		*p;
	int		len;
	int		ret;

	p = xframe->packets;
	xframe_end = p + XFRAME_LEN(xframe);
	do {
		pack = (xpacket_t *)p;
		len = XPACKET_LEN(pack);
		/* Sanity checks */
		if(unlikely(XPACKET_OP(pack) == XPROTO_NAME(GLOBAL,PCM_READ))) {
			static int	rate_limit;

			if((rate_limit++ % 1003) == 0) {
				XBUS_DBG(GENERAL, xbus, "A PCM packet within a Non-PCM xframe\n");
				dump_xframe("In Non-PCM xframe", xbus, xframe, print_dbg);
			}
			ret = -EPROTO;
			goto out;
		}
		p += len;
		if(p > xframe_end || len < RPACKET_HEADERSIZE) {
			static int	rate_limit;

			if((rate_limit++ % 1003) == 0) {
				XBUS_NOTICE(xbus, "Invalid packet length %d\n", len);
				dump_xframe("BAD LENGTH", xbus, xframe, print_dbg);
			}
			ret = -EPROTO;
			goto out;
		}
		ret = packet_process(xbus, pack);
		if(unlikely(ret < 0))
			break;
	} while(p < xframe_end);
out:
	FREE_RECV_XFRAME(xbus, xframe);
	return ret;
}

int xframe_receive(xbus_t *xbus, xframe_t *xframe)
{
	int		ret = 0;
	struct timeval	now;
	struct timeval	tv_received;
	int		usec;

	if(XFRAME_LEN(xframe) < RPACKET_HEADERSIZE) {
		static int	rate_limit;

		if((rate_limit++ % 1003) == 0) {
			XBUS_NOTICE(xbus, "short xframe\n");
			dump_xframe("short xframe", xbus, xframe, print_dbg);
		}
		FREE_RECV_XFRAME(xbus, xframe);
		return -EPROTO;
	}
	if(!XBUS_GET(xbus)) {
		XBUS_DBG(GENERAL, xbus, "Dropped xframe. Is shutting down.\n");
		return -ENODEV;
	}
	tv_received = xframe->tv_received;
	/*
	 * We want to check that xframes do not mix PCM and other commands
	 */
	if(XPACKET_IS_PCM((xpacket_t *)xframe->packets))
		xframe_receive_pcm(xbus, xframe);
	else {
		XBUS_COUNTER(xbus, RX_CMD)++;
		ret = xframe_receive_cmd(xbus, xframe);
	}
	/* Calculate total processing time */
	do_gettimeofday(&now);
	usec = (now.tv_sec - tv_received.tv_sec) * 1000000 +
		now.tv_usec - tv_received.tv_usec;
	if(usec > xbus->max_rx_process)
		xbus->max_rx_process = usec;
	XBUS_PUT(xbus);
	return ret;
}

#define	VERBOSE_DEBUG		1
#define	ERR_REPORT_LIMIT	20

void dump_packet(const char *msg, const xpacket_t *packet, bool print_dbg)
{
	byte	op = XPACKET_OP(packet);
	byte	*addr = (byte *)&XPACKET_ADDR(packet);

	if(!print_dbg)
		return;
	printk(KERN_DEBUG "%s: XPD=%1X-%1X%c (0x%X) OP=0x%02X LEN=%d",
			msg,
			XPACKET_ADDR_UNIT(packet),
			XPACKET_ADDR_SUBUNIT(packet),
			(XPACKET_ADDR_SYNC(packet))?'+':' ',
			*addr,
			op,
			XPACKET_LEN(packet));
#if VERBOSE_DEBUG
	{
		int i;
		byte	*p = (byte *)packet;

		printk(" BYTES: ");
		for(i = 0; i < XPACKET_LEN(packet); i++) {
			static int limiter = 0;

			if(i >= sizeof(xpacket_t)) {
				if(limiter < ERR_REPORT_LIMIT) {
					ERR("%s: length overflow i=%d > sizeof(xpacket_t)=%lu\n",
						__FUNCTION__, i+1, (long)sizeof(xpacket_t));
				} else if(limiter == ERR_REPORT_LIMIT) {
					ERR("%s: error packet #%d... squelsh reports.\n",
						__FUNCTION__, limiter);
				}
				limiter++;
				break;
			}
			if (print_dbg)
				printk("%02X ", p[i]);
		}
	}
#endif
	printk("\n");
}

void dump_reg_cmd(const char msg[], const reg_cmd_t *regcmd, bool writing)
{
	char		action;
	byte		chipsel;

	if(regcmd->bytes != sizeof(*regcmd) - 1) {	/* The size byte is not included */
		NOTICE("%s: Wrong size: regcmd->bytes = %d\n", __FUNCTION__, regcmd->bytes);
		return;
	}
	if(writing && (REG_FIELD(regcmd, chipsel) & 0x80))
		ERR("Sending register command with reading bit ON\n");
	action = (writing) ? 'W' : 'R';
	chipsel = REG_FIELD(regcmd, chipsel) & ~0x80;
	if(REG_FIELD(regcmd, do_subreg)) {
		DBG(GENERAL, "%s: %d %cS %02X %02X [%02X] # data_high=%X m=%d eof=%d\n", msg, chipsel, action,
			REG_FIELD(regcmd, regnum),
			REG_FIELD(regcmd, subreg),
			REG_FIELD(regcmd, data_low),
			REG_FIELD(regcmd, data_high),
			regcmd->multibyte, regcmd->eoframe);
	} else if(REG_FIELD(regcmd, regnum) == 0x1E) {
		DBG(GENERAL, "%s: %d %cI %02X [%02X %02X] # m=%d eof=%d\n", msg, chipsel, action,
			REG_FIELD(regcmd, subreg),
			REG_FIELD(regcmd, data_low),
			REG_FIELD(regcmd, data_high),
			regcmd->multibyte, regcmd->eoframe);
	} else {
		DBG(GENERAL, "%s: %d %cD %02X [%02X] # data_high=%X m=%d eof=%d\n", msg, chipsel, action,
			REG_FIELD(regcmd, regnum),
			REG_FIELD(regcmd, data_low),
			REG_FIELD(regcmd, data_high),
			regcmd->multibyte, regcmd->eoframe);
	}
}

const char *xproto_name(xpd_type_t xpd_type)
{
	const xproto_table_t	*proto_table;

	BUG_ON(xpd_type >= XPD_TYPE_NOMODULE);
	proto_table = xprotocol_tables[xpd_type];
	if(!proto_table)
		return NULL;
	return proto_table->name;
}

#define	CHECK_XOP(f)	\
		if(!(xops)->f) { \
			ERR("%s: missing xmethod %s [%s (%d)]\n", __FUNCTION__, #f, name, type);	\
			return -EINVAL;	\
		}

int xproto_register(const xproto_table_t *proto_table)
{
	int		type;
	const char	*name;
	const xops_t	*xops;
	
	BUG_ON(!proto_table);
	type = proto_table->type;
	name = proto_table->name;
	if(type >= XPD_TYPE_NOMODULE) {
		NOTICE("%s: Bad xproto type %d\n", __FUNCTION__, type);
		return -EINVAL;
	}
	DBG(GENERAL, "%s (%d)\n", name, type);
	if(xprotocol_tables[type])
		NOTICE("%s: overriding registration of %s (%d)\n", __FUNCTION__, name, type);
	xops = &proto_table->xops;
	CHECK_XOP(card_new);
	CHECK_XOP(card_init);
	CHECK_XOP(card_remove);
	CHECK_XOP(card_tick);
	CHECK_XOP(card_pcm_fromspan);
	CHECK_XOP(card_pcm_tospan);
	CHECK_XOP(card_zaptel_preregistration);
	CHECK_XOP(card_zaptel_postregistration);
	CHECK_XOP(card_hooksig);
	// CHECK_XOP(card_ioctl);	// optional method -- call after testing
	CHECK_XOP(card_register_reply);
	CHECK_XOP(XPD_STATE);
	CHECK_XOP(RING);
	CHECK_XOP(RELAY_OUT);

	xprotocol_tables[type] = proto_table;
	return 0;
}

void xproto_unregister(const xproto_table_t *proto_table)
{
	int		type;
	const char	*name;
	
	BUG_ON(!proto_table);
	type = proto_table->type;
	name = proto_table->name;
	DBG(GENERAL, "%s (%d)\n", name, type);
	if(type >= XPD_TYPE_NOMODULE) {
		NOTICE("%s: Bad xproto type %s (%d)\n", __FUNCTION__, name, type);
		return;
	}
	if(!xprotocol_tables[type])
		NOTICE("%s: xproto type %s (%d) is already unregistered\n", __FUNCTION__, name, type);
	xprotocol_tables[type] = NULL;
}

EXPORT_SYMBOL(dump_packet);
EXPORT_SYMBOL(dump_reg_cmd);
EXPORT_SYMBOL(xframe_receive);
EXPORT_SYMBOL(notify_bad_xpd);
EXPORT_SYMBOL(valid_xpd_addr);
EXPORT_SYMBOL(xproto_global_entry);
EXPORT_SYMBOL(xproto_card_entry);
EXPORT_SYMBOL(xproto_name);
EXPORT_SYMBOL(xproto_register);
EXPORT_SYMBOL(xproto_unregister);
