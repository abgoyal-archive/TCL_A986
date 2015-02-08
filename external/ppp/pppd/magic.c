

#define RCSID	"$Id: magic.c,v 1.11 2003/06/11 23:56:26 paulus Exp $"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>

#include "pppd.h"
#include "magic.h"

static const char rcsid[] = RCSID;

extern long mrand48 __P((void));
extern void srand48 __P((long));

void
magic_init()
{
    long seed;
    struct timeval t;

    gettimeofday(&t, NULL);
    seed = get_host_seed() ^ t.tv_sec ^ t.tv_usec ^ getpid();
    srand48(seed);
}

u_int32_t
magic()
{
    return (u_int32_t) mrand48();
}

void
random_bytes(unsigned char *buf, int len)
{
	int i;

	for (i = 0; i < len; ++i)
		buf[i] = mrand48() >> 24;
}

#ifdef NO_DRAND48

double
drand48()
{
    return (double)random() / (double)0x7fffffffL; /* 2**31-1 */
}

long
mrand48()
{
    return random();
}

void
srand48(seedval)
long seedval;
{
    srandom((int)seedval);
}

#endif
