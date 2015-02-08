

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/ipproto.c,v 1.3.2.3 2005/09/20 06:05:37 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include "interface.h"
#include "ipproto.h"

struct tok ipproto_values[] = {
    { IPPROTO_HOPOPTS, "Options" },
    { IPPROTO_ICMP, "ICMP" },
    { IPPROTO_IGMP, "IGMP" },
    { IPPROTO_IPV4, "IPIP" },
    { IPPROTO_TCP, "TCP" },
    { IPPROTO_EGP, "EGP" },
    { IPPROTO_PIGP, "IGRP" },
    { IPPROTO_UDP, "UDP" },
    { IPPROTO_DCCP, "DCCP" },
    { IPPROTO_IPV6, "IPv6" },
    { IPPROTO_ROUTING, "Routing" },
    { IPPROTO_FRAGMENT, "Fragment" },
    { IPPROTO_RSVP, "RSVP" },
    { IPPROTO_GRE, "GRE" },
    { IPPROTO_ESP, "ESP" },
    { IPPROTO_AH, "AH" },
    { IPPROTO_MOBILE, "Mobile IP" },
    { IPPROTO_ICMPV6, "ICMPv6" },
    { IPPROTO_MOBILITY_OLD, "Mobile IP (old)" },
    { IPPROTO_EIGRP, "EIGRP" },
    { IPPROTO_OSPF, "OSPF" },
    { IPPROTO_PIM, "PIM" },
    { IPPROTO_IPCOMP, "Compressed IP" },
    { IPPROTO_VRRP, "VRRP" },
    { IPPROTO_PGM, "PGM" },
    { IPPROTO_SCTP, "SCTP" },
    { IPPROTO_MOBILITY, "Mobility" },
    { 0, NULL }
};

