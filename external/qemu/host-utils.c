

#include <stdlib.h>
#include <stdint.h>
#include "host-utils.h"

//#define DEBUG_MULDIV

/* Long integer helpers */
#if !defined(__x86_64__)
static void add128 (uint64_t *plow, uint64_t *phigh, uint64_t a, uint64_t b)
{
    *plow += a;
    /* carry test */
    if (*plow < a)
        (*phigh)++;
    *phigh += b;
}

static void neg128 (uint64_t *plow, uint64_t *phigh)
{
    *plow = ~*plow;
    *phigh = ~*phigh;
    add128(plow, phigh, 1, 0);
}

static void mul64 (uint64_t *plow, uint64_t *phigh, uint64_t a, uint64_t b)
{
    uint32_t a0, a1, b0, b1;
    uint64_t v;

    a0 = a;
    a1 = a >> 32;

    b0 = b;
    b1 = b >> 32;

    v = (uint64_t)a0 * (uint64_t)b0;
    *plow = v;
    *phigh = 0;

    v = (uint64_t)a0 * (uint64_t)b1;
    add128(plow, phigh, v << 32, v >> 32);

    v = (uint64_t)a1 * (uint64_t)b0;
    add128(plow, phigh, v << 32, v >> 32);

    v = (uint64_t)a1 * (uint64_t)b1;
    *phigh += v;
}

/* Unsigned 64x64 -> 128 multiplication */
void mulu64 (uint64_t *plow, uint64_t *phigh, uint64_t a, uint64_t b)
{
    mul64(plow, phigh, a, b);
#if defined(DEBUG_MULDIV)
    printf("mulu64: 0x%016llx * 0x%016llx = 0x%016llx%016llx\n",
           a, b, *phigh, *plow);
#endif
}

/* Signed 64x64 -> 128 multiplication */
void muls64 (uint64_t *plow, uint64_t *phigh, int64_t a, int64_t b)
{
    int sa, sb;

    sa = (a < 0);
    if (sa)
        a = -a;
    sb = (b < 0);
    if (sb)
        b = -b;
    mul64(plow, phigh, a, b);
    if (sa ^ sb) {
        neg128(plow, phigh);
    }
#if defined(DEBUG_MULDIV)
    printf("muls64: 0x%016llx * 0x%016llx = 0x%016llx%016llx\n",
           a, b, *phigh, *plow);
#endif
}
#endif /* !defined(__x86_64__) */
