

/* Cisco Hot Standby Router Protocol (HSRP). */

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-hsrp.c,v 1.9.2.1 2005/05/06 07:57:17 guy Exp $";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <stdio.h>

#include "interface.h"
#include "addrtoname.h"

/* HSRP op code types. */
static const char *op_code_str[] = {
	"hello",
	"coup",
	"resign"
};

/* HSRP states and associated names. */
static struct tok states[] = {
	{  0, "initial" },
	{  1, "learn" },
	{  2, "listen" },
	{  4, "speak" },
	{  8, "standby" },
	{ 16, "active" },
	{  0, NULL }
};


#define HSRP_AUTH_SIZE	8

/* HSRP protocol header. */
struct hsrp {
	u_int8_t	hsrp_version;
	u_int8_t	hsrp_op_code;
	u_int8_t	hsrp_state;
	u_int8_t	hsrp_hellotime;
	u_int8_t	hsrp_holdtime;
	u_int8_t	hsrp_priority;
	u_int8_t	hsrp_group;
	u_int8_t	hsrp_reserved;
	u_int8_t	hsrp_authdata[HSRP_AUTH_SIZE];
	struct in_addr	hsrp_virtaddr;
};

void
hsrp_print(register const u_int8_t *bp, register u_int len)
{
	struct hsrp *hp = (struct hsrp *) bp;

	TCHECK(hp->hsrp_version);
	printf("HSRPv%d", hp->hsrp_version);
	if (hp->hsrp_version != 0)
		return;
	TCHECK(hp->hsrp_op_code);
	printf("-");
	printf("%s ", tok2strary(op_code_str, "unknown (%d)", hp->hsrp_op_code));
	printf("%d: ", len);
	TCHECK(hp->hsrp_state);
	printf("state=%s ", tok2str(states, "Unknown (%d)", hp->hsrp_state));
	TCHECK(hp->hsrp_group);
	printf("group=%d ", hp->hsrp_group);
	TCHECK(hp->hsrp_reserved);
	if (hp->hsrp_reserved != 0) {
		printf("[reserved=%d!] ", hp->hsrp_reserved);
	}
	TCHECK(hp->hsrp_virtaddr);
	printf("addr=%s", ipaddr_string(&hp->hsrp_virtaddr));
	if (vflag) {
		printf(" hellotime=");
		relts_print(hp->hsrp_hellotime);
		printf(" holdtime=");
		relts_print(hp->hsrp_holdtime);
		printf(" priority=%d", hp->hsrp_priority);
		printf(" auth=\"");
		if (fn_printn(hp->hsrp_authdata, sizeof(hp->hsrp_authdata),
		    snapend)) {
			printf("\"");
			goto trunc;
		}
		printf("\"");
	}
	return;
trunc:
	printf("[|hsrp]");
}
