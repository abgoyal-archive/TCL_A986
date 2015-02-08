

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/missing/datalinks.c,v 1.3 2003/11/16 09:36:47 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <pcap.h>
#include <stdlib.h>
#include <stdio.h>

#include "pcap-missing.h"

int
pcap_list_datalinks(pcap_t *p, int **dlt_buffer)
{
	/*
	 * This platform doesn't support changing the DLT for an
	 * interface.  Return a list of DLTs containing only the
	 * DLT this device supports.
	 */
	*dlt_buffer = (int*)malloc(sizeof(**dlt_buffer));
	if (*dlt_buffer == NULL)
		return (-1);
	**dlt_buffer = pcap_datalink(p);
	return (1);
}
