

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-netbios.c,v 1.20 2003/11/16 09:36:29 guy Exp $";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "interface.h"
#include "addrtoname.h"
#include "netbios.h"
#include "extract.h"

void
netbios_print(struct p8022Hdr *nb, u_int length)
{
	if (length < p8022Size) {
		(void)printf(" truncated-netbios %d", length);
		return;
	}

	if (nb->flags == UI) {
	    (void)printf("802.1 UI ");
	} else {
	    (void)printf("802.1 CONN ");
	}

	if ((u_char *)(nb + 1) > snapend) {
		printf(" [|netbios]");
		return;
	}

}

#ifdef never
	(void)printf("%s.%d > ",
		     ipxaddr_string(EXTRACT_32BITS(ipx->srcNet), ipx->srcNode),
		     EXTRACT_16BITS(ipx->srcSkt));

	(void)printf("%s.%d:",
		     ipxaddr_string(EXTRACT_32BITS(ipx->dstNet), ipx->dstNode),
		     EXTRACT_16BITS(ipx->dstSkt));

	if ((u_char *)(ipx + 1) > snapend) {
		printf(" [|ipx]");
		return;
	}

	/* take length from ipx header */
	length = EXTRACT_16BITS(&ipx->length);

	ipx_decode(ipx, (u_char *)ipx + ipxSize, length - ipxSize);
#endif

