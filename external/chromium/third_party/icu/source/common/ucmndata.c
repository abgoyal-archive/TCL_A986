


#include "unicode/utypes.h"
#include "unicode/udata.h"
#include "cstring.h"
#include "ucmndata.h"
#include "udatamem.h"

#if defined(UDATA_DEBUG) || defined(UDATA_DEBUG_DUMP)
#   include <stdio.h>
#endif

U_CFUNC uint16_t
udata_getHeaderSize(const DataHeader *udh) {
    if(udh==NULL) {
        return 0;
    } else if(udh->info.isBigEndian==U_IS_BIG_ENDIAN) {
        /* same endianness */
        return udh->dataHeader.headerSize;
    } else {
        /* opposite endianness */
        uint16_t x=udh->dataHeader.headerSize;
        return (uint16_t)((x<<8)|(x>>8));
    }
}

U_CFUNC uint16_t
udata_getInfoSize(const UDataInfo *info) {
    if(info==NULL) {
        return 0;
    } else if(info->isBigEndian==U_IS_BIG_ENDIAN) {
        /* same endianness */
        return info->size;
    } else {
        /* opposite endianness */
        uint16_t x=info->size;
        return (uint16_t)((x<<8)|(x>>8));
    }
}

typedef struct {
    const char       *entryName;
    const DataHeader *pHeader;
} PointerTOCEntry;


typedef struct  {
    uint32_t          count;
    uint32_t          reserved;
    PointerTOCEntry   entry[2];   /* Actual size is from count. */
}  PointerTOC;


/* definition of OffsetTOC struct types moved to ucmndata.h */

static uint32_t offsetTOCEntryCount(const UDataMemory *pData) {
    int32_t          retVal=0;
    const UDataOffsetTOC *toc = (UDataOffsetTOC *)pData->toc;
    if (toc != NULL) {
        retVal = toc->count;
    }
    return retVal;
}


static const DataHeader *
offsetTOCLookupFn(const UDataMemory *pData,
                  const char *tocEntryName,
                  int32_t *pLength,
                  UErrorCode *pErrorCode) {
    const UDataOffsetTOC  *toc = (UDataOffsetTOC *)pData->toc;
    if(toc!=NULL) {
        const char *base=(const char *)toc;
        uint32_t start, limit, number, lastNumber;
        int32_t strResult;
        const UDataOffsetTOCEntry *entry;

        /* perform a binary search for the data in the common data's table of contents */
#if defined (UDATA_DEBUG_DUMP)
        /* list the contents of the TOC each time .. not recommended */
        for(start=0;start<toc->count;start++) {
          fprintf(stderr, "\tx%d: %s\n", start, &base[toc->entry[start].nameOffset]);
        }
#endif

        start=0;
        limit=toc->count;         /* number of names in this table of contents */
        lastNumber=limit;
        entry=toc->entry;
        for (;;) {
            number = (start+limit)/2;
            if (lastNumber == number) { /* Have we moved? */
                break;  /* We haven't moved, and it wasn't found; */
                        /* or the empty stub common data library was used during build. */
            }
            lastNumber = number;
            strResult = uprv_strcmp(tocEntryName, base+entry[number].nameOffset);
            if(strResult<0) {
                limit=number;
            } else if (strResult>0) {
                start=number;
            }
            else {
                /* found it */
#ifdef UDATA_DEBUG
                fprintf(stderr, "%s: Found.\n", tocEntryName);
#endif
                entry += number; /* Alias the entry to the current entry. */
                if((number+1) < toc->count) {
                    *pLength = (int32_t)(entry[1].dataOffset - entry->dataOffset);
                } else {
                    *pLength = -1;
                }
                return (const DataHeader *)(base+entry->dataOffset);
            }
        }
#ifdef UDATA_DEBUG
        fprintf(stderr, "%s: Not found.\n", tocEntryName);
#endif
        return NULL;
    } else {
#ifdef UDATA_DEBUG
        fprintf(stderr, "returning header\n");
#endif

        return pData->pHeader;
    }
}


static uint32_t pointerTOCEntryCount(const UDataMemory *pData) {
    const PointerTOC *toc = (PointerTOC *)pData->toc;
    return (uint32_t)((toc != NULL) ? (toc->count) : 0);
}


static const DataHeader *pointerTOCLookupFn(const UDataMemory *pData,
                   const char *name,
                   int32_t *pLength,
                   UErrorCode *pErrorCode) {
    if(pData->toc!=NULL) {
        const PointerTOC *toc = (PointerTOC *)pData->toc;
        uint32_t start, limit, number, lastNumber;
        int32_t strResult;

#if defined (UDATA_DEBUG_DUMP)
        /* list the contents of the TOC each time .. not recommended */
        for(start=0;start<toc->count;start++) {
            fprintf(stderr, "\tx%d: %s\n", start, toc->entry[start].entryName);
        }
#endif

        /* perform a binary search for the data in the common data's table of contents */
        start=0;
        limit=toc->count;
        lastNumber=limit;

        for (;;) {
            number = (start+limit)/2;
            if (lastNumber == number) { /* Have we moved? */
                break;  /* We haven't moved, and it wasn't found, */
                        /* or the empty stub common data library was used during build. */
            }
            lastNumber = number;
            strResult = uprv_strcmp(name, toc->entry[number].entryName);
            if(strResult<0) {
                limit=number;
            } else if (strResult>0) {
                start=number;
            }
            else {
                /* found it */
#ifdef UDATA_DEBUG
                fprintf(stderr, "%s: Found.\n", toc->entry[number].entryName);
#endif
                *pLength=-1;
                return UDataMemory_normalizeDataPointer(toc->entry[number].pHeader);
            }
        }
#ifdef UDATA_DEBUG
        fprintf(stderr, "%s: Not found.\n", name);
#endif
        return NULL;
    } else {
        return pData->pHeader;
    }
}

static const commonDataFuncs CmnDFuncs = {offsetTOCLookupFn,  offsetTOCEntryCount};
static const commonDataFuncs ToCPFuncs = {pointerTOCLookupFn, pointerTOCEntryCount};



void udata_checkCommonData(UDataMemory *udm, UErrorCode *err) {
    if (U_FAILURE(*err)) {
        return;
    }

    if(!(udm->pHeader->dataHeader.magic1==0xda &&
        udm->pHeader->dataHeader.magic2==0x27 &&
        udm->pHeader->info.isBigEndian==U_IS_BIG_ENDIAN &&
        udm->pHeader->info.charsetFamily==U_CHARSET_FAMILY)
        ) {
        /* header not valid */
        *err=U_INVALID_FORMAT_ERROR;
    }
    else if (udm->pHeader->info.dataFormat[0]==0x43 &&
        udm->pHeader->info.dataFormat[1]==0x6d &&
        udm->pHeader->info.dataFormat[2]==0x6e &&
        udm->pHeader->info.dataFormat[3]==0x44 &&
        udm->pHeader->info.formatVersion[0]==1
        ) {
        /* dataFormat="CmnD" */
        udm->vFuncs = &CmnDFuncs;
        udm->toc=(const char *)udm->pHeader+udata_getHeaderSize(udm->pHeader);
    }
    else if(udm->pHeader->info.dataFormat[0]==0x54 &&
        udm->pHeader->info.dataFormat[1]==0x6f &&
        udm->pHeader->info.dataFormat[2]==0x43 &&
        udm->pHeader->info.dataFormat[3]==0x50 &&
        udm->pHeader->info.formatVersion[0]==1
        ) {
        /* dataFormat="ToCP" */
        udm->vFuncs = &ToCPFuncs;
        udm->toc=(const char *)udm->pHeader+udata_getHeaderSize(udm->pHeader);
    }
    else {
        /* dataFormat not recognized */
        *err=U_INVALID_FORMAT_ERROR;
    }

    if (U_FAILURE(*err)) {
        /* If the data is no good and we memory-mapped it ourselves,
         *  close the memory mapping so it doesn't leak.  Note that this has
         *  no effect on non-memory mapped data, other than clearing fields in udm.
         */
        udata_close(udm);
    }
}

