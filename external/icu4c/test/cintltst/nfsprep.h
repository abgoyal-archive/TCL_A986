#ifndef _NFSPREP_H
#define _NFSPREP_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_IDNA

#include "unicode/ustring.h"
#include "unicode/usprep.h"
#include <stdlib.h>
#include <string.h>


/* this enum must be kept in syn with NFS4DataFileNames array in nfsprep.c */
enum NFS4ProfileState{
    NFS4_CS_PREP_CS,
    NFS4_CS_PREP_CI,
    NFS4_CIS_PREP,
    NFS4_MIXED_PREP_PREFIX,
    NFS4_MIXED_PREP_SUFFIX
};

typedef enum NFS4ProfileState NFS4ProfileState;

int32_t
nfs4_prepare(const char* src, int32_t srcLength,
                  char* dest, int32_t destCapacity,
                  NFS4ProfileState state,
                  UParseError* parseError,
                  UErrorCode*  status);

int32_t
nfs4_mixed_prepare( const char* src, int32_t srcLength,
                    char* dest, int32_t destCapacity,
                    UParseError* parseError,
                    UErrorCode*  status);

int32_t
nfs4_cis_prepare(   const char* src, int32_t srcLength,
                    char* dest, int32_t destCapacity,
                    UParseError* parseError,
                    UErrorCode*  status);

int32_t
nfs4_cs_prepare(    const char* src, int32_t srcLength,
                    char* dest, int32_t destCapacity,
                    UBool isCaseSensitive,
                    UParseError* parseError,
                    UErrorCode*  status);
#endif

#endif
