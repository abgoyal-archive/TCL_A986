
#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-msdp.c,v 1.7 2005/04/06 21:32:41 mcr Exp $";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <stdio.h>
#include <stdlib.h>

#include "interface.h"
#include "addrtoname.h"
#include "extract.h"

#define MSDP_TYPE_MAX	7

void
msdp_print(const unsigned char *sp, u_int length)
{
	unsigned int type, len;

	TCHECK2(*sp, 3);
	/* See if we think we're at the beginning of a compound packet */
	type = *sp;
	len = EXTRACT_16BITS(sp + 1);
	if (len > 1500 || len < 3 || type == 0 || type > MSDP_TYPE_MAX)
		goto trunc;	/* not really truncated, but still not decodable */
	(void)printf(" msdp:");
	while (length > 0) {
		TCHECK2(*sp, 3);
		type = *sp;
		len = EXTRACT_16BITS(sp + 1);
		if (len > 1400 || vflag)
			printf(" [len %u]", len);
		if (len < 3)
			goto trunc;
		sp += 3;
		length -= 3;
		switch (type) {
		case 1:	/* IPv4 Source-Active */
		case 3: /* IPv4 Source-Active Response */
			if (type == 1)
				(void)printf(" SA");
			else
				(void)printf(" SA-Response");
			TCHECK(*sp);
			(void)printf(" %u entries", *sp);
			if ((u_int)((*sp * 12) + 8) < len) {
				(void)printf(" [w/data]");
				if (vflag > 1) {
					(void)printf(" ");
					ip_print(gndo, sp + *sp * 12 + 8 - 3,
					         len - (*sp * 12 + 8));
				}
			}
			break;
		case 2:
			(void)printf(" SA-Request");
			TCHECK2(*sp, 5);
			(void)printf(" for %s", ipaddr_string(sp + 1));
			break;
		case 4:
			(void)printf(" Keepalive");
			if (len != 3)
				(void)printf("[len=%d] ", len);
			break;
		case 5:
			(void)printf(" Notification");
			break;
		default:
			(void)printf(" [type=%d len=%d]", type, len);
			break;
		}
		sp += (len - 3);
		length -= (len - 3);
	}
	return;
trunc:
	(void)printf(" [|msdp]");
}

