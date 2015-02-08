

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-eap.c,v 1.3 2004/04/23 19:03:39 mcr Exp $";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <stdio.h>
#include <string.h>

#include "netdissect.h"
#include "addrtoname.h"
#include "extract.h"
#include "ether.h"

struct eap_packet_t {
	unsigned char	code;
	unsigned char	id;
	unsigned char	length[2];
	unsigned char	data[1];
};

void
eap_print(netdissect_options *ndo,
	  register const u_char *cp,
	  u_int length _U_)
{
	const struct eap_packet_t *eap;

	eap = (const struct eap_packet_t *)cp;
	ND_TCHECK(eap->data);

        ND_PRINT((ndo, "EAP code=%u id=%u length=%u ", 
		  eap->code, eap->id, (eap->length[0]<<8) + eap->length[1]));

        if (!ndo->ndo_vflag)
            return;

trunc:
	;
}

