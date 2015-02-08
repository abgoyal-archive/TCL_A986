
/* -*- Mode: c; tab-width: 8; indent-tabs-mode: 1; c-basic-offset: 8; -*- */

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/libpcap/fad-null.c,v 1.2 2003/11/15 23:23:58 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <pcap.h>

int
pcap_findalldevs(pcap_if_t **alldevsp, char *errbuf)
{
	/*
	 * Succeed, but don't return any interfaces; we return only those
	 * we can open, and we can't open any if there's no support
	 * for packet capture.
	 */
	*alldevsp = NULL;
	return (0);
}
