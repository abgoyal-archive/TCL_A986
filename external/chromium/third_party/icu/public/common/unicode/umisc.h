
#ifndef UMISC_H
#define UMISC_H

#include "unicode/utypes.h"


U_CDECL_BEGIN

typedef struct UFieldPosition {
  /**
   * The field 
   * @stable ICU 2.0
   */
  int32_t field;
  /**
   * The start of the text range containing field 
   * @stable ICU 2.0
   */
  int32_t beginIndex;
  /** 
   * The limit of the text range containing field 
   * @stable ICU 2.0
   */
  int32_t endIndex;
} UFieldPosition;

#if !UCONFIG_NO_SERVICE
typedef const void* URegistryKey;
#endif

U_CDECL_END

#endif
