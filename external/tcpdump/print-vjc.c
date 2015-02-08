

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-vjc.c,v 1.15 2004/03/25 03:31:17 mcr Exp $ (LBL)";
#endif

#include <tcpdump-stdinc.h>

#include <pcap.h>
#include <stdio.h>

#include "interface.h"
#include "addrtoname.h"

#include "slcompress.h"
#include "ppp.h"

int
vjc_print(register const char *bp, u_short proto _U_)
{
	int i;

	switch (bp[0] & 0xf0) {
	case TYPE_IP:
		if (eflag)
			printf("(vjc type=IP) ");
		return PPP_IP;
	case TYPE_UNCOMPRESSED_TCP:
		if (eflag)
			printf("(vjc type=raw TCP) ");
		return PPP_IP;
	case TYPE_COMPRESSED_TCP:
		if (eflag)
			printf("(vjc type=compressed TCP) ");
		for (i = 0; i < 8; i++) {
			if (bp[1] & (0x80 >> i))
				printf("%c", "?CI?SAWU"[i]);
		}
		if (bp[1])
			printf(" ");
		printf("C=0x%02x ", bp[2]);
		printf("sum=0x%04x ", *(u_short *)&bp[3]);
		return -1;
	case TYPE_ERROR:
		if (eflag)
			printf("(vjc type=error) ");
		return -1;
	default:
		if (eflag)
			printf("(vjc type=0x%02x) ", bp[0] & 0xf0);
		return -1;
	}
}
