

#define RCSID	"$Id: eui64.c,v 1.6 2002/12/04 23:03:32 paulus Exp $"

#include "pppd.h"

static const char rcsid[] = RCSID;

char *
eui64_ntoa(e)
    eui64_t e;
{
    static char buf[32];

    snprintf(buf, 32, "%02x%02x:%02x%02x:%02x%02x:%02x%02x",
	     e.e8[0], e.e8[1], e.e8[2], e.e8[3], 
	     e.e8[4], e.e8[5], e.e8[6], e.e8[7]);
    return buf;
}
