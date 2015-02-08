
/*	$NetBSD: print-ah.c,v 1.4 1996/05/20 00:41:16 fvdl Exp $	*/


#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-ah.c,v 1.22 2003/11/19 00:36:06 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <stdio.h>

#include "ah.h"

#include "interface.h"
#include "addrtoname.h"
#include "extract.h"

int
ah_print(register const u_char *bp)
{
	register const struct ah *ah;
	register const u_char *ep;
	int sumlen;
	u_int32_t spi;

	ah = (const struct ah *)bp;
	ep = snapend;		/* 'ep' points to the end of available data. */

	TCHECK(*ah);

	sumlen = ah->ah_len << 2;
	spi = EXTRACT_32BITS(&ah->ah_spi);

	printf("AH(spi=0x%08x", spi);
	if (vflag)
		printf(",sumlen=%d", sumlen);
	printf(",seq=0x%x", EXTRACT_32BITS(ah + 1));
	if (bp + sizeof(struct ah) + sumlen > ep)
		fputs("[truncated]", stdout);
	fputs("): ", stdout);

	return sizeof(struct ah) + sumlen;
 trunc:
	fputs("[|AH]", stdout);
	return -1;
}
