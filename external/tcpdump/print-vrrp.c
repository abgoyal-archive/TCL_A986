

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-vrrp.c,v 1.9.2.1 2005/05/06 07:57:20 guy Exp $";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <stdio.h>
#include <stdlib.h>

#include "interface.h"
#include "extract.h"
#include "addrtoname.h"


/* Type */
#define	VRRP_TYPE_ADVERTISEMENT	1

static const struct tok type2str[] = {
	{ VRRP_TYPE_ADVERTISEMENT,	"Advertisement"	},
	{ 0,				NULL		}
};

/* Auth Type */
#define	VRRP_AUTH_NONE		0
#define	VRRP_AUTH_SIMPLE	1
#define	VRRP_AUTH_AH		2

static const struct tok auth2str[] = {
	{ VRRP_AUTH_NONE,		"none"		},
	{ VRRP_AUTH_SIMPLE,		"simple"	},
	{ VRRP_AUTH_AH,			"ah"		},
	{ 0,				NULL		}
};

void
vrrp_print(register const u_char *bp, register u_int len, int ttl)
{
	int version, type, auth_type;
	const char *type_s;

	TCHECK(bp[0]);
	version = (bp[0] & 0xf0) >> 4;
	type = bp[0] & 0x0f;
	type_s = tok2str(type2str, "unknown type (%u)", type);
	printf("VRRPv%u, %s", version, type_s);
	if (ttl != 255)
		printf(", (ttl %u)", ttl);
	if (version != 2 || type != VRRP_TYPE_ADVERTISEMENT)
		return;
	TCHECK(bp[2]);
	printf(", vrid %u, prio %u", bp[1], bp[2]);
	TCHECK(bp[5]);
	auth_type = bp[4];
	printf(", authtype %s", tok2str(auth2str, NULL, auth_type));
	printf(", intvl %us, length %u", bp[5],len);
	if (vflag) {
		int naddrs = bp[3];
		int i;
		char c;

		if (TTEST2(bp[0], len) && in_cksum((const u_short*)bp, len, 0))
			printf(", (bad vrrp cksum %x)",
				EXTRACT_16BITS(&bp[6]));
		printf(", addrs");
		if (naddrs > 1)
			printf("(%d)", naddrs);
		printf(":");
		c = ' ';
		bp += 8;
		for (i = 0; i < naddrs; i++) {
			TCHECK(bp[3]);
			printf("%c%s", c, ipaddr_string(bp));
			c = ',';
			bp += 4;
		}
		if (auth_type == VRRP_AUTH_SIMPLE) { /* simple text password */
			TCHECK(bp[7]);
			printf(" auth \"");
			if (fn_printn(bp, 8, snapend)) {
				printf("\"");
				goto trunc;
			}
			printf("\"");
		}
	}
	return;
trunc:
	printf("[|vrrp]");
}
