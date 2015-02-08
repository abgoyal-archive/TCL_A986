#ifndef UCAT_H
#define UCAT_H

#include "unicode/utypes.h"
#include "unicode/ures.h"


U_CDECL_BEGIN

typedef UResourceBundle* u_nl_catd;

U_STABLE u_nl_catd U_EXPORT2
u_catopen(const char* name, const char* locale, UErrorCode* ec);

U_STABLE void U_EXPORT2
u_catclose(u_nl_catd catd);

U_STABLE const UChar* U_EXPORT2
u_catgets(u_nl_catd catd, int32_t set_num, int32_t msg_num,
          const UChar* s,
          int32_t* len, UErrorCode* ec);

U_CDECL_END

#endif /*UCAT_H*/
/*eof*/
