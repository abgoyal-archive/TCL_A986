/*	$NetBSD: ipsec_get_policylen.c,v 1.6.6.1 2007/08/01 11:52:17 vanhu Exp $	*/

/*	$KAME: ipsec_get_policylen.c,v 1.5 2000/05/07 05:25:03 itojun Exp $	*/


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/types.h>
#include <sys/param.h>
#include <sys/socket.h>

#include PATH_IPSEC_H

#include <net/pfkeyv2.h>

#include "libpfkey.h"
#include "ipsec_strerror.h"

int
ipsec_get_policylen(policy)
	ipsec_policy_t policy;
{
	return policy ? PFKEY_EXTLEN(policy) : -1;
}
