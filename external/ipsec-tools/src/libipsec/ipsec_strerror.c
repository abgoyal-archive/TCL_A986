/*	$NetBSD: ipsec_strerror.c,v 1.4.6.1 2007/08/01 11:52:17 vanhu Exp $	*/

/*	$KAME: ipsec_strerror.c,v 1.7 2000/07/30 00:45:12 itojun Exp $	*/


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/types.h>
#include <sys/param.h>

#include <string.h>
#include PATH_IPSEC_H

#include "ipsec_strerror.h"

int __ipsec_errcode;

static const char *ipsec_errlist[] = {
"Success",					/*EIPSEC_NO_ERROR*/
"Not supported",				/*EIPSEC_NOT_SUPPORTED*/
"Invalid argument",				/*EIPSEC_INVAL_ARGUMENT*/
"Invalid sadb message",				/*EIPSEC_INVAL_SADBMSG*/
"Invalid version",				/*EIPSEC_INVAL_VERSION*/
"Invalid security policy",			/*EIPSEC_INVAL_POLICY*/
"Invalid address specification",		/*EIPSEC_INVAL_ADDRESS*/
"Invalid ipsec protocol",			/*EIPSEC_INVAL_PROTO*/
"Invalid ipsec mode",				/*EIPSEC_INVAL_MODE*/
"Invalid ipsec level",				/*EIPSEC_INVAL_LEVEL*/
"Invalid SA type",				/*EIPSEC_INVAL_SATYPE*/
"Invalid message type",				/*EIPSEC_INVAL_MSGTYPE*/
"Invalid extension type",			/*EIPSEC_INVAL_EXTTYPE*/
"Invalid algorithm type",			/*EIPSEC_INVAL_ALGS*/
"Invalid key length",				/*EIPSEC_INVAL_KEYLEN*/
"Invalid address family",			/*EIPSEC_INVAL_FAMILY*/
"Invalid prefix length",			/*EIPSEC_INVAL_PREFIXLEN*/
"Invalid direciton",				/*EIPSEC_INVAL_DIR*/
"SPI range violation",				/*EIPSEC_INVAL_SPI*/
"No protocol specified",			/*EIPSEC_NO_PROTO*/
"No algorithm specified",			/*EIPSEC_NO_ALGS*/
"No buffers available",				/*EIPSEC_NO_BUFS*/
"Must get supported algorithms list first",	/*EIPSEC_DO_GET_SUPP_LIST*/
"Protocol mismatch",				/*EIPSEC_PROTO_MISMATCH*/
"Family mismatch",				/*EIPSEC_FAMILY_MISMATCH*/
"Too few arguments",				/*EIPSEC_FEW_ARGUMENTS*/
NULL,						/*EIPSEC_SYSTEM_ERROR*/
"Priority offset not in valid range [-2147483647, 2147483648]",	/*EIPSEC_INVAL_PRIORITY_OFFSET*/
"Priority offset from base not in valid range [0, 1073741823] for negative offsets and [0, 1073741824] for positive offsets", /* EIPSEC_INVAL_PRIORITY_OFFSET */
"Policy priority not compiled in",	/*EIPSEC_PRIORITY_NOT_COMPILED*/
"Unknown error",				/*EIPSEC_MAX*/
};

const char *ipsec_strerror(void)
{
	if (__ipsec_errcode < 0 || __ipsec_errcode > EIPSEC_MAX)
		__ipsec_errcode = EIPSEC_MAX;

	return ipsec_errlist[__ipsec_errcode];
}

void __ipsec_set_strerror(const char *str)
{
	__ipsec_errcode = EIPSEC_SYSTEM_ERROR;
	ipsec_errlist[EIPSEC_SYSTEM_ERROR] = str;

	return;
}
