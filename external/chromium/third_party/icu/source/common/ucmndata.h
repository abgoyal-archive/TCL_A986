


#ifndef __UCMNDATA_H__
#define __UCMNDATA_H__

#include "unicode/udata.h"
#include "umapfile.h"


#define COMMON_DATA_NAME U_ICUDATA_NAME

typedef struct  {
    uint16_t    headerSize;
    uint8_t     magic1;
    uint8_t     magic2;
} MappedData;


typedef struct  {
    MappedData  dataHeader;
    UDataInfo   info;
} DataHeader;

typedef struct {
    DataHeader hdr;
    char padding[8];
    uint32_t count, reserved;
    /*
    const struct {
    const char *const name; 
    const void *const data;
    } toc[1];
    */
   int   fakeNameAndData[4];       /* TODO:  Change this header type from */
                                   /*        pointerTOC to OffsetTOC.     */
} ICU_Data_Header;

typedef struct {
    uint32_t nameOffset;
    uint32_t dataOffset;
} UDataOffsetTOCEntry;

typedef struct {
    uint32_t count;
    UDataOffsetTOCEntry entry[2];    /* Actual size of array is from count. */
} UDataOffsetTOC;

U_CFUNC uint16_t
udata_getHeaderSize(const DataHeader *udh);

U_CFUNC uint16_t
udata_getInfoSize(const UDataInfo *info);


typedef const DataHeader *
(* LookupFn)(const UDataMemory *pData,
             const char *tocEntryName,
             int32_t *pLength,
             UErrorCode *pErrorCode);

typedef uint32_t
(* NumEntriesFn)(const UDataMemory *pData);

typedef struct {
    LookupFn      Lookup;
    NumEntriesFn  NumEntries; 
} commonDataFuncs;


void   udata_checkCommonData(UDataMemory *pData, UErrorCode *pErrorCode);


#endif
