

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-sip.c,v 1.1 2004/07/27 17:04:20 hannes Exp $";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <stdio.h>
#include <stdlib.h>

#include "interface.h"
#include "extract.h"

#include "udp.h"

void
sip_print(register const u_char *pptr, register u_int len)
{
    u_int idx;

    printf("SIP, length: %u%s", len, vflag ? "\n\t" : "");

    /* in non-verbose mode just lets print the protocol and length */
    if (vflag < 1)
        return;

    for (idx = 0; idx < len; idx++) {
        if (EXTRACT_16BITS(pptr+idx) != 0x0d0a) { /* linefeed ? */
            safeputchar(*(pptr+idx));
        } else {
            printf("\n\t");
            idx+=1;
        }
    }

    /* do we want to see an additionally hexdump ? */
    if (vflag> 1)
        print_unknown_data(pptr,"\n\t",len);

    return;
}
