
#ifndef _BITSETV_H
#define _BITSETV_H

#include "bitset.h"

typedef bitset * bitsetv;

extern bitsetv bitsetv_alloc (bitset_bindex, bitset_bindex, enum bitset_type);

extern bitsetv bitsetv_create (bitset_bindex, bitset_bindex, unsigned int);

/* Free vector of bitsets.  */
extern void bitsetv_free (bitsetv);

/* Zero vector of bitsets.  */
extern void bitsetv_zero (bitsetv);

/* Set vector of bitsets.  */
extern void bitsetv_ones (bitsetv);

extern void bitsetv_transitive_closure (bitsetv);

extern void bitsetv_reflexive_transitive_closure (bitsetv);

/* Dump vector of bitsets.  */
extern void bitsetv_dump (FILE *, const char *, const char *, bitsetv);

/* Function to debug vector of bitsets from debugger.  */
extern void debug_bitsetv (bitsetv);

#endif  /* _BITSETV_H  */
