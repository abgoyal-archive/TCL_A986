
#ifndef __WRITESRC_H__
#define __WRITESRC_H__

#include <stdio.h>
#include "unicode/utypes.h"
#include "utrie2.h"

U_CAPI FILE * U_EXPORT2
usrc_create(const char *path, const char *filename);

U_CAPI FILE * U_EXPORT2
usrc_createTextData(const char *path, const char *filename);

U_CAPI void U_EXPORT2
usrc_writeArray(FILE *f,
                const char *prefix,
                const void *p, int32_t width, int32_t length,
                const char *postfix);

U_CAPI void U_EXPORT2
usrc_writeUTrie2Arrays(FILE *f,
                       const char *indexPrefix, const char *dataPrefix,
                       const UTrie2 *pTrie,
                       const char *postfix);

U_CAPI void U_EXPORT2
usrc_writeUTrie2Struct(FILE *f,
                       const char *prefix,
                       const UTrie2 *pTrie,
                       const char *indexName, const char *dataName,
                       const char *postfix);

#endif
