
#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-sunatm.c,v 1.8 2004/03/17 23:24:38 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>
 
struct mbuf;
struct rtentry;
 
#include <stdio.h>
#include <pcap.h>

#include "interface.h"
#include "extract.h"
#include "addrtoname.h"

#include "atm.h"
#include "atmuni31.h"

/* SunATM header for ATM packet */
#define DIR_POS		0	/* Direction (0x80 = transmit, 0x00 = receive) */
#define VPI_POS		1	/* VPI */
#define VCI_POS		2	/* VCI */
#define PKT_BEGIN_POS   4	/* Start of the ATM packet */

/* Protocol type values in the bottom for bits of the byte at SUNATM_DIR_POS. */
#define PT_LANE		0x01	/* LANE */
#define PT_LLC		0x02	/* LLC encapsulation */

u_int
sunatm_if_print(const struct pcap_pkthdr *h, const u_char *p)
{
	u_int caplen = h->caplen;
	u_int length = h->len;
	u_short vci;
	u_char vpi;
	u_int traftype;

	if (caplen < PKT_BEGIN_POS) {
		printf("[|atm]");
		return (caplen);
	}

	if (eflag) {
		if (p[DIR_POS] & 0x80)
			printf("Tx: ");
		else
			printf("Rx: ");
	}

	switch (p[DIR_POS] & 0x0f) {

	case PT_LANE:
		traftype = ATM_LANE;
		break;

	case PT_LLC:
		traftype = ATM_LLC;
		break;

	default:
		traftype = ATM_UNKNOWN;
		break;
	}

	vci = EXTRACT_16BITS(&p[VCI_POS]);
	vpi = p[VPI_POS];

	p += PKT_BEGIN_POS;
	caplen -= PKT_BEGIN_POS;
	length -= PKT_BEGIN_POS;
	atm_print(vpi, vci, traftype, p, length, caplen);

	return (PKT_BEGIN_POS);
}
