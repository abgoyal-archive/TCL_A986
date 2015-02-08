
#ifndef _BITSET_STATS_H
#define _BITSET_STATS_H

#include "bbitset.h"

extern bool bitset_stats_enabled;

extern enum bitset_type bitset_stats_type_get (bitset);

extern size_t bitset_stats_bytes (void);

extern bitset bitset_stats_init (bitset, bitset_bindex, enum bitset_type);

#endif
