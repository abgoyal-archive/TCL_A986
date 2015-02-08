
#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-ap1394.c,v 1.3.2.1 2005/07/07 01:24:33 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <stdio.h>
#include <pcap.h>

#include "interface.h"
#include "addrtoname.h"
#include "ethertype.h"

#define FIREWIRE_EUI64_LEN	8
struct firewire_header {
	u_char  firewire_dhost[FIREWIRE_EUI64_LEN];
	u_char  firewire_shost[FIREWIRE_EUI64_LEN];
	u_short firewire_type;
};

#define FIREWIRE_HDRLEN		18

static inline void
ap1394_hdr_print(register const u_char *bp, u_int length)
{
	register const struct firewire_header *fp;
	fp = (const struct firewire_header *)bp;

	(void)printf("%s > %s",
		     linkaddr_string(fp->firewire_dhost, FIREWIRE_EUI64_LEN),
		     linkaddr_string(fp->firewire_shost, FIREWIRE_EUI64_LEN));

	if (!qflag) {
		(void)printf(", ethertype %s (0x%04x)",
			       tok2str(ethertype_values,"Unknown", ntohs(fp->firewire_type)),
                               ntohs(fp->firewire_type));	      
        } else {
                (void)printf(", %s", tok2str(ethertype_values,"Unknown Ethertype (0x%04x)", ntohs(fp->firewire_type)));  
        }

	(void)printf(", length %u: ", length);
}

u_int
ap1394_if_print(const struct pcap_pkthdr *h, const u_char *p)
{
	u_int length = h->len;
	u_int caplen = h->caplen;
	struct firewire_header *fp;
	u_short ether_type;
	u_short extracted_ether_type;

	if (caplen < FIREWIRE_HDRLEN) {
		printf("[|ap1394]");
		return FIREWIRE_HDRLEN;
	}

	if (eflag)
		ap1394_hdr_print(p, length);

	length -= FIREWIRE_HDRLEN;
	caplen -= FIREWIRE_HDRLEN;
	fp = (struct firewire_header *)p;
	p += FIREWIRE_HDRLEN;

	ether_type = ntohs(fp->firewire_type);

	extracted_ether_type = 0;
	if (ether_encap_print(ether_type, p, length, caplen,
	    &extracted_ether_type) == 0) {
		/* ether_type not known, print raw packet */
		if (!eflag)
			ap1394_hdr_print((u_char *)fp, length + FIREWIRE_HDRLEN);

		if (!suppress_default_print)
			default_print(p, caplen);
	} 

	return FIREWIRE_HDRLEN;
}
