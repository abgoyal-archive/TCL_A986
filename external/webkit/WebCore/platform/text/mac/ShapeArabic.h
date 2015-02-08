

#ifndef ShapeArabic_h
#define ShapeArabic_h

#if USE(ATSUI)

#include <unicode/ushape.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t shapeArabic(const UChar *source, int32_t sourceLength, UChar *dest, int32_t destCapacity, uint32_t options, UErrorCode *pErrorCode);

#ifdef __cplusplus
}
#endif

#endif // USE(ATSUI)
#endif // ShapeArabic_h
