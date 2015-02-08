
/* @(#) $Header: /tcpdump/master/tcpdump/af.h,v 1.3.2.1 2007/02/26 13:33:17 hannes Exp $ (LBL) */


extern struct tok af_values[];
extern struct tok bsd_af_values[];

/* RFC1700 address family numbers */
#define AFNUM_INET	1
#define AFNUM_INET6	2
#define AFNUM_NSAP	3
#define AFNUM_HDLC	4
#define AFNUM_BBN1822	5
#define AFNUM_802	6
#define AFNUM_E163	7
#define AFNUM_E164	8
#define AFNUM_F69	9
#define AFNUM_X121	10
#define AFNUM_IPX	11
#define AFNUM_ATALK	12
#define AFNUM_DECNET	13
#define AFNUM_BANYAN	14
#define AFNUM_E164NSAP	15
#define AFNUM_VPLS      25
/* draft-kompella-ppvpn-l2vpn */
#define AFNUM_L2VPN     196 /* still to be approved by IANA */

#define BSD_AFNUM_INET		2
#define BSD_AFNUM_NS		6		/* XEROX NS protocols */
#define BSD_AFNUM_ISO		7
#define BSD_AFNUM_APPLETALK	16
#define BSD_AFNUM_IPX		23
#define BSD_AFNUM_INET6_BSD	24	/* OpenBSD (and probably NetBSD), BSD/OS */
#define BSD_AFNUM_INET6_FREEBSD	28
#define BSD_AFNUM_INET6_DARWIN	30
