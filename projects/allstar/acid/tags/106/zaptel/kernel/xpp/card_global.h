#ifndef	CARD_GLOBAL_H
#define	CARD_GLOBAL_H
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

#include "xdefs.h"
#include "xbus-pcm.h"

enum global_opcodes {
	XPROTO_NAME(GLOBAL, DESC_REQ)		= 0x04,
	XPROTO_NAME(GLOBAL, DEV_DESC)		= 0x05,
	XPROTO_NAME(GLOBAL, REGISTER_REQUEST)	= 0x0F,
	XPROTO_NAME(GLOBAL, REGISTER_REPLY)	= 0x10,
/**/
	XPROTO_NAME(GLOBAL, PCM_WRITE)		= 0x11,
	XPROTO_NAME(GLOBAL, PCM_READ)		= 0x12,
/**/
	XPROTO_NAME(GLOBAL, SYNC_SOURCE)	= 0x19,
	XPROTO_NAME(GLOBAL, SYNC_REPLY)		= 0x1A,
/**/
	XPROTO_NAME(GLOBAL, ERROR_CODE)		= 0x22,
	XPROTO_NAME(GLOBAL, RESET_SYNC_COUNTERS)	= 0x23,
	XPROTO_NAME(GLOBAL, NULL_REPLY)		= 0xFE,
};

DEF_RPACKET_DATA(GLOBAL, NULL_REPLY);
DEF_RPACKET_DATA(GLOBAL, DESC_REQ);
DEF_RPACKET_DATA(GLOBAL, DEV_DESC,
	byte		rev;		/* Revision number */
	byte		type:4;		/* LSB: 1 - to_phone, 0 - to_line */
	byte		subtype:4;	/* default 0 */
	xpp_line_t	line_status;	/* hook/ring status, depending on unit */
	);
DEF_RPACKET_DATA(GLOBAL, REGISTER_REQUEST,
	reg_cmd_t	reg_cmd;
	);
DEF_RPACKET_DATA(GLOBAL, PCM_WRITE,
	xpp_line_t	lines;
	byte		pcm[PCM_CHUNKSIZE];
	);
DEF_RPACKET_DATA(GLOBAL, PCM_READ,
	xpp_line_t	lines;
	byte		pcm[PCM_CHUNKSIZE];
	);
DEF_RPACKET_DATA(GLOBAL, SYNC_SOURCE,
	byte		sync_mode;
	byte		drift;
	);
DEF_RPACKET_DATA(GLOBAL, SYNC_REPLY,
	byte		sync_mode;
	byte		drift;
	);
DEF_RPACKET_DATA(GLOBAL, REGISTER_REPLY,
	reg_cmd_t	regcmd;
	);
DEF_RPACKET_DATA(GLOBAL, RESET_SYNC_COUNTERS,
	byte		mask;
	);
DEF_RPACKET_DATA(GLOBAL, ERROR_CODE,
	byte		errorcode;
	union {
		reg_cmd_t	bad_spi_cmd;
	} info;
	);

/* 0x04 */ DECLARE_CMD(GLOBAL, DESC_REQ, int xpd_num);
/* 0x19 */ DECLARE_CMD(GLOBAL, SYNC_SOURCE, enum sync_mode mode, int drift);
/* 0x23 */ DECLARE_CMD(GLOBAL, RESET_SYNC_COUNTERS);

int xpp_register_request(xbus_t *xbus, xpd_t *xpd,
	byte chipsel, bool writing, bool do_subreg, byte regnum, byte subreg, byte data_low, byte data_high);
extern xproto_table_t PROTO_TABLE(GLOBAL);
int run_initialize_registers(xpd_t *xpd);

#endif	/* CARD_GLOBAL_H */
