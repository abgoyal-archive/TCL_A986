
/*	$NetBSD: print-mobile.c,v 1.2 1998/09/30 08:57:01 hwr Exp $ */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifndef lint
static const char rcsid[] _U_ =
     "@(#) $Header: /tcpdump/master/tcpdump/print-mobile.c,v 1.15 2004/03/24 01:58:14 guy Exp $";
#endif

#include <tcpdump-stdinc.h>

#include <stdio.h>

#include "interface.h"
#include "addrtoname.h"
#include "extract.h"		/* must come after interface.h */

#define MOBILE_SIZE (8)

struct mobile_ip {
	u_int16_t proto;
	u_int16_t hcheck;
	u_int32_t odst;
	u_int32_t osrc;
};

#define OSRC_PRES	0x0080	/* old source is present */

void
mobile_print(const u_char *bp, u_int length)
{
	const u_char *cp = bp +8 ;
	const struct mobile_ip *mob;
	u_short proto,crc;
	u_char osp =0;			/* old source address present */

	mob = (const struct mobile_ip *)bp;

	if (length < MOBILE_SIZE || !TTEST(*mob)) {
		fputs("[|mobile]", stdout);
		return;
	}
	fputs("mobile: ", stdout);

	proto = EXTRACT_16BITS(&mob->proto);
	crc =  EXTRACT_16BITS(&mob->hcheck);
	if (proto & OSRC_PRES) {
		osp=1;
		cp +=4 ;
	}

	if (osp)  {
		fputs("[S] ",stdout);
		if (vflag)
			(void)printf("%s ",ipaddr_string(&mob->osrc));
	} else {
		fputs("[] ",stdout);
	}
	if (vflag) {
		(void)printf("> %s ",ipaddr_string(&mob->odst));
		(void)printf("(oproto=%d)",proto>>8);
	}
	if (in_cksum((u_short *)mob, osp ? 12 : 8, 0)!=0) {
		(void)printf(" (bad checksum %d)",crc);
	}

	return;
}
