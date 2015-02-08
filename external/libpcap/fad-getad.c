
/* -*- Mode: c; tab-width: 8; indent-tabs-mode: 1; c-basic-offset: 8; -*- */

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/libpcap/fad-getad.c,v 1.10.2.2 2007/09/14 00:45:17 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <net/if.h>

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ifaddrs.h>

#include "pcap-int.h"

#ifdef HAVE_OS_PROTO_H
#include "os-proto.h"
#endif

#ifdef AF_PACKET
# ifdef __Lynx__
#  include <netpacket/if_packet.h>	/* LynxOS */
# else
#  include <linux/if_packet.h>		/* Linux */
# endif
#endif

#ifndef SA_LEN
#ifdef HAVE_SOCKADDR_SA_LEN
#define SA_LEN(addr)	((addr)->sa_len)
#else /* HAVE_SOCKADDR_SA_LEN */
#ifdef HAVE_SOCKADDR_STORAGE
static size_t
get_sa_len(struct sockaddr *addr)
{
	switch (addr->sa_family) {

#ifdef AF_INET
	case AF_INET:
		return (sizeof (struct sockaddr_in));
#endif

#ifdef AF_INET6
	case AF_INET6:
		return (sizeof (struct sockaddr_in6));
#endif

#ifdef AF_PACKET
	case AF_PACKET:
		return (sizeof (struct sockaddr_ll));
#endif

	default:
		return (sizeof (struct sockaddr));
	}
}
#define SA_LEN(addr)	(get_sa_len(addr))
#else /* HAVE_SOCKADDR_STORAGE */
#define SA_LEN(addr)	(sizeof (struct sockaddr))
#endif /* HAVE_SOCKADDR_STORAGE */
#endif /* HAVE_SOCKADDR_SA_LEN */
#endif /* SA_LEN */

int
pcap_findalldevs(pcap_if_t **alldevsp, char *errbuf)
{
	pcap_if_t *devlist = NULL;
	struct ifaddrs *ifap, *ifa;
	struct sockaddr *addr, *netmask, *broadaddr, *dstaddr;
	size_t addr_size, broadaddr_size, dstaddr_size;
	int ret = 0;
	char *p, *q;

	/*
	 * Get the list of interface addresses.
	 *
	 * Note: this won't return information about interfaces
	 * with no addresses; are there any such interfaces
	 * that would be capable of receiving packets?
	 * (Interfaces incapable of receiving packets aren't
	 * very interesting from libpcap's point of view.)
	 *
	 * LAN interfaces will probably have link-layer
	 * addresses; I don't know whether all implementations
	 * of "getifaddrs()" now, or in the future, will return
	 * those.
	 */
	if (getifaddrs(&ifap) != 0) {
		(void)snprintf(errbuf, PCAP_ERRBUF_SIZE,
		    "getifaddrs: %s", pcap_strerror(errno));
		return (-1);
	}
	for (ifa = ifap; ifa != NULL; ifa = ifa->ifa_next) {
		/*
		 * Is this interface up?
		 */
		if (!(ifa->ifa_flags & IFF_UP)) {
			/*
			 * No, so don't add it to the list.
			 */
			continue;
		}

		/*
		 * "ifa_addr" was apparently null on at least one
		 * interface on some system.
		 *
		 * "ifa_broadaddr" may be non-null even on
		 * non-broadcast interfaces, and was null on
		 * at least one OpenBSD 3.4 system on at least
		 * one interface with IFF_BROADCAST set.
		 *
		 * "ifa_dstaddr" was, on at least one FreeBSD 4.1
		 * system, non-null on a non-point-to-point
		 * interface.
		 *
		 * Therefore, we supply the address and netmask only
		 * if "ifa_addr" is non-null (if there's no address,
		 * there's obviously no netmask), and supply the
		 * broadcast and destination addresses if the appropriate
		 * flag is set *and* the appropriate "ifa_" entry doesn't
		 * evaluate to a null pointer.
		 */
		if (ifa->ifa_addr != NULL) {
			addr = ifa->ifa_addr;
			addr_size = SA_LEN(addr);
			netmask = ifa->ifa_netmask;
		} else {
			addr = NULL;
			addr_size = 0;
			netmask = NULL;
		}
		if (ifa->ifa_flags & IFF_BROADCAST &&
		    ifa->ifa_broadaddr != NULL) {
			broadaddr = ifa->ifa_broadaddr;
			broadaddr_size = SA_LEN(broadaddr);
		} else {
			broadaddr = NULL;
			broadaddr_size = 0;
		}
		if (ifa->ifa_flags & IFF_POINTOPOINT &&
		    ifa->ifa_dstaddr != NULL) {
			dstaddr = ifa->ifa_dstaddr;
			dstaddr_size = SA_LEN(ifa->ifa_dstaddr);
		} else {
			dstaddr = NULL;
			dstaddr_size = 0;
		}

		/*
		 * If this entry has a colon followed by a number at
		 * the end, we assume it's a logical interface.  Those
		 * are just the way you assign multiple IP addresses to
		 * a real interface on Linux, so an entry for a logical
		 * interface should be treated like the entry for the
		 * real interface; we do that by stripping off the ":"
		 * and the number.
		 *
		 * XXX - should we do this only on Linux?
		 */
		p = strchr(ifa->ifa_name, ':');
		if (p != NULL) {
			/*
			 * We have a ":"; is it followed by a number?
			 */
			q = p + 1;
			while (isdigit((unsigned char)*q))
				q++;
			if (*q == '\0') {
				/*
				 * All digits after the ":" until the end.
				 * Strip off the ":" and everything after
				 * it.
				 */
			       *p = '\0';
			}
		}

		/*
		 * Add information for this address to the list.
		 */
		if (add_addr_to_iflist(&devlist, ifa->ifa_name,
		    ifa->ifa_flags, addr, addr_size, netmask, addr_size,
		    broadaddr, broadaddr_size, dstaddr, dstaddr_size,
		    errbuf) < 0) {
			ret = -1;
			break;
		}
	}

	freeifaddrs(ifap);

	if (ret != -1) {
		/*
		 * We haven't had any errors yet; do any platform-specific
		 * operations to add devices.
		 */
		if (pcap_platform_finddevs(&devlist, errbuf) < 0)
			ret = -1;
	}

	if (ret == -1) {
		/*
		 * We had an error; free the list we've been constructing.
		 */
		if (devlist != NULL) {
			pcap_freealldevs(devlist);
			devlist = NULL;
		}
	}

	*alldevsp = devlist;
	return (ret);
}
