#include "unicode/ucat.h"
#include "unicode/ustring.h"
#include "cstring.h"
#include "uassert.h"

/* Separator between set_num and msg_num */
static const char SEPARATOR = '%';

#define MAX_KEY_LEN (24)

static char*
_catkey(char* buffer, int32_t set_num, int32_t msg_num) {
    int32_t i = 0;
    i = T_CString_integerToString(buffer, set_num, 10);
    buffer[i++] = SEPARATOR;
    T_CString_integerToString(buffer+i, msg_num, 10);
    return buffer;
}

U_CAPI u_nl_catd U_EXPORT2
u_catopen(const char* name, const char* locale, UErrorCode* ec) {
    return (u_nl_catd) ures_open(name, locale, ec);
}

U_CAPI void U_EXPORT2
u_catclose(u_nl_catd catd) {
    ures_close((UResourceBundle*) catd); /* may be NULL */
}

U_CAPI const UChar* U_EXPORT2
u_catgets(u_nl_catd catd, int32_t set_num, int32_t msg_num,
          const UChar* s,
          int32_t* len, UErrorCode* ec) {

    char key[MAX_KEY_LEN];
    const UChar* result;

    if (ec == NULL || U_FAILURE(*ec)) {
        goto ERROR;
    }

    result = ures_getStringByKey((const UResourceBundle*) catd,
                                 _catkey(key, set_num, msg_num),
                                 len, ec);
    if (U_FAILURE(*ec)) {
        goto ERROR;
    }

    return result;

 ERROR:
    /* In case of any failure, return s */
    if (len != NULL) {
        *len = u_strlen(s);
    }
    return s;
}

/*eof*/
