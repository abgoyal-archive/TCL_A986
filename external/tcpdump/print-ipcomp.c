

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-ipcomp.c,v 1.20 2003/11/19 00:36:08 guy Exp $";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include <tcpdump-stdinc.h>

#include <stdio.h>

struct ipcomp {
	u_int8_t comp_nxt;	/* Next Header */
	u_int8_t comp_flags;	/* Length of data, in 32bit */
	u_int16_t comp_cpi;	/* Compression parameter index */
};

#if defined(HAVE_LIBZ) && defined(HAVE_ZLIB_H)
#include <zlib.h>
#endif

#include "interface.h"
#include "addrtoname.h"
#include "extract.h"

int
ipcomp_print(register const u_char *bp, int *nhdr _U_)
{
	register const struct ipcomp *ipcomp;
	register const u_char *ep;
	u_int16_t cpi;
#if defined(HAVE_LIBZ) && defined(HAVE_ZLIB_H)
	int advance;
#endif

	ipcomp = (struct ipcomp *)bp;
	cpi = EXTRACT_16BITS(&ipcomp->comp_cpi);

	/* 'ep' points to the end of available data. */
	ep = snapend;

	if ((u_char *)(ipcomp + 1) >= ep - sizeof(struct ipcomp)) {
		fputs("[|IPCOMP]", stdout);
		goto fail;
	}
	printf("IPComp(cpi=0x%04x)", cpi);

#if defined(HAVE_LIBZ) && defined(HAVE_ZLIB_H)
	if (1)
		goto fail;

	/*
	 * We may want to decompress the packet here.  Packet buffer
	 * management is a headache (if we decompress, packet will become
	 * larger).
	 */
	if (nhdr)
		*nhdr = ipcomp->comp_nxt;
	advance = sizeof(struct ipcomp);

	printf(": ");
	return advance;

#endif
fail:
	return -1;
}
