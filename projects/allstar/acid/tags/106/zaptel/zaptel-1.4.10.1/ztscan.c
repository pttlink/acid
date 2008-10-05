/*
 * Scan and output information about Zaptel spans and ports.
 * 
 * Written by Brandon Kruse <bkruse@digium.com>
 * and Kevin P. Fleming <kpfleming@digium.com>
 * Copyright (C) 2007 Digium, Inc.
 *
 * Based on zttool written by Mark Spencer <markster@digium.com>
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under thet erms of the GNU General Public License as published by
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

#include <stdio.h> 
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>

#ifdef STANDALONE_ZAPATA
#include "kernel/zaptel.h"
#else
#include <zaptel/zaptel.h>
#endif

void print_span(int spanno, int ctl)
{
	struct zt_spaninfo spaninfo;
	int y;
	struct zt_params params;
	unsigned int basechan = 1;
	char buf[100];
	char alarms[50];

	memset(&spaninfo, 0, sizeof(spaninfo));
	spaninfo.spanno = spanno;
	if (ioctl(ctl, ZT_SPANSTAT, &spaninfo))
		return;

	alarms[0] = '\0';
	if (spaninfo.alarms) {
		if (spaninfo.alarms & ZT_ALARM_BLUE)
			strcat(alarms,"BLU/");
		if (spaninfo.alarms & ZT_ALARM_YELLOW)
			strcat(alarms, "YEL/");
		if (spaninfo.alarms & ZT_ALARM_RED)
			strcat(alarms, "RED/");
		if (spaninfo.alarms & ZT_ALARM_LOOPBACK)
			strcat(alarms,"LB/");
		if (spaninfo.alarms & ZT_ALARM_RECOVER)
			strcat(alarms,"REC/");
		if (spaninfo.alarms & ZT_ALARM_NOTOPEN)
			strcat(alarms, "NOP/");
		if (!strlen(alarms))
			strcat(alarms, "UUU/");
		if (strlen(alarms)) {
			/* Strip trailing / */
			alarms[strlen(alarms)-1]='\0';
		}
	} else {
		if (spaninfo.numchans)
			strcpy(alarms, "OK");
		else
			strcpy(alarms, "UNCONFIGURED");
	}

	fprintf(stdout, "[%d]\n", spanno);
	fprintf(stdout, "active=yes\n");
	fprintf(stdout, "alarms=%s\n", alarms);
	fprintf(stdout, "description=%s\n", spaninfo.desc);
	fprintf(stdout, "name=%s\n", spaninfo.name);
	fprintf(stdout, "manufacturer=%s\n", spaninfo.manufacturer);
	fprintf(stdout, "devicetype=%s\n", spaninfo.devicetype);
	fprintf(stdout, "location=%s\n", spaninfo.location);
	fprintf(stdout, "basechan=%d\n", basechan);
	fprintf(stdout, "totchans=%d\n", spaninfo.totalchans);
	fprintf(stdout, "irq=%d\n", spaninfo.irq);
	y = basechan;
	memset(&params, 0, sizeof(params));
	params.channo = y;
	if (ioctl(ctl, ZT_GET_PARAMS, &params)) {
		basechan += spaninfo.totalchans;
		return;
	}

	if (params.sigcap & (__ZT_SIG_DACS |  ZT_SIG_CAS)) {
		/* this is a digital span */
		fprintf(stdout, "type=digital-%s\n", spaninfo.spantype);
		fprintf(stdout, "syncsrc=%d\n", spaninfo.syncsrc);
		fprintf(stdout, "lbo=%s\n", spaninfo.lboname);
		fprintf(stdout, "coding_opts=");
		buf[0] = '\0';
		if (spaninfo.linecompat & ZT_CONFIG_B8ZS) strcat(buf, "B8ZS,");
		if (spaninfo.linecompat & ZT_CONFIG_AMI) strcat(buf, "AMI,");
		if (spaninfo.linecompat & ZT_CONFIG_HDB3) strcat(buf, "HDB3,");
		buf[strlen(buf) - 1] = '\0';
		fprintf(stdout, "%s\n", buf);
		fprintf(stdout, "framing_opts=");
		buf[0] = '\0';
		if (spaninfo.linecompat & ZT_CONFIG_ESF) strcat(buf, "ESF,");
		if (spaninfo.linecompat & ZT_CONFIG_D4) strcat(buf, "D4,");
		if (spaninfo.linecompat & ZT_CONFIG_CCS) strcat(buf, "CCS,");
		if (spaninfo.linecompat & ZT_CONFIG_CRC4) strcat(buf, "CRC4,");
		buf[strlen(buf) - 1] = '\0';
		fprintf(stdout, "%s\n", buf);
		fprintf(stdout, "coding=");
		if (spaninfo.lineconfig & ZT_CONFIG_B8ZS) fprintf(stdout, "B8ZS");
		else if (spaninfo.lineconfig & ZT_CONFIG_AMI) fprintf(stdout, "AMI");
		else if (spaninfo.lineconfig & ZT_CONFIG_HDB3) fprintf(stdout, "HDB3");
		fprintf(stdout, "\n");
		fprintf(stdout, "framing=");
		if (spaninfo.lineconfig & ZT_CONFIG_ESF) fprintf(stdout, "ESF");
		else if (spaninfo.lineconfig & ZT_CONFIG_D4) fprintf(stdout, "D4");
		else if (spaninfo.lineconfig & ZT_CONFIG_CCS) fprintf(stdout, "CCS");
		else if (spaninfo.lineconfig & ZT_CONFIG_CRC4) fprintf(stdout, "/CRC4");
		fprintf(stdout, "\n");
	} else {
		/* this is an analog span */
		fprintf(stdout, "type=analog\n");
		for (y = basechan; y < (basechan + spaninfo.totalchans); y++) {
			memset(&params, 0, sizeof(params));
			params.channo = y;
			if (ioctl(ctl, ZT_GET_PARAMS, &params)) {
				fprintf(stdout, "port=%d,unknown\n", y);
				return;
			};
			fprintf(stdout, "port=%d,", y);
			switch (params.sigcap & (__ZT_SIG_FXO | __ZT_SIG_FXS)) {
				case __ZT_SIG_FXO:
					fprintf(stdout, "FXS");
					break;
				case __ZT_SIG_FXS:
					fprintf(stdout, "FXO");
					break;
				default:
					fprintf(stdout, "none");
			}
			if (params.sigcap & ZT_SIG_BROKEN)
				fprintf(stdout, " FAILED");
			fprintf(stdout, "\n");
		}
	}

	basechan += spaninfo.totalchans;
}

int main(int argc, char *argv[])
{
	int ctl;
	int x, spanno;

	if ((ctl = open("/dev/zap/ctl", O_RDWR)) < 0) {
		fprintf(stderr, "Unable to open /dev/zap/ctl: %s\n", strerror(errno));
		exit(1);
	}
	
	/* If no parameters: loop over all spans */
	if (argc == 1) {
		for (spanno = 1; spanno < ZT_MAX_SPANS; spanno++)
			print_span(spanno, ctl);
		return 0;
	}

	/* Parameters provided: use them as span numbers */
	for (x=1; argv[x]; x++) {
		/* We can use atoi because span numbers must be > 0 */
		int spanno = atoi(argv[x]);
		if (spanno <= 0) {
			fprintf(stderr, "%s: Error: Invalid span number %d (%s). Aborting\n",
					argv[0], spanno, argv[x]);
			exit(2);
		}
		print_span(spanno, ctl);
	}

	return 0;
}
