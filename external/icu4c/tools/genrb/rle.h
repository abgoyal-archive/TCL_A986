
#ifndef RLE_H
#define RLE_H 1

#include "unicode/utypes.h"
#include "unicode/ustring.h"

U_CDECL_BEGIN
int32_t
byteArrayToRLEString(const uint8_t* src,int32_t srcLen, uint16_t* buffer,int32_t bufLen, UErrorCode* status);


int32_t
usArrayToRLEString(const uint16_t* src,int32_t srcLen,uint16_t* buffer, int32_t bufLen,UErrorCode* status);

int32_t
rleStringToByteArray(uint16_t* src, int32_t srcLen, uint8_t* target, int32_t tgtLen, UErrorCode* status); 
int32_t
rleStringToUCharArray(uint16_t* src, int32_t srcLen, uint16_t* target, int32_t tgtLen, UErrorCode* status);

U_CDECL_END

#endif
