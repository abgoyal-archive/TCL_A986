
#ifndef LOCMAP_H
#define LOCMAP_H

#include "unicode/utypes.h"

#define LANGUAGE_LCID(hostID) (uint16_t)(0x03FF & hostID)

U_CAPI const char *uprv_convertToPosix(uint32_t hostid, UErrorCode* status);

/* Don't call this function directly. Use uloc_getLCID instead. */
U_CAPI uint32_t uprv_convertToLCID(const char *langID, const char* posixID, UErrorCode* status);

#endif /* LOCMAP_H */

