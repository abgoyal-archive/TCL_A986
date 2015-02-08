
#ifndef PARSEERR_H
#define PARSEERR_H

#include "unicode/utypes.h"


enum { U_PARSE_CONTEXT_LEN = 16 };

typedef struct UParseError {

    /**
     * The line on which the error occured.  If the parser uses this
     * field, it sets it to the line number of the source text line on
     * which the error appears, which will be be a value >= 1.  If the
     * parse does not support line numbers, the value will be <= 0.
     * @stable ICU 2.0
     */
    int32_t        line;

    /**
     * The character offset to the error.  If the line field is >= 1,
     * then this is the offset from the start of the line.  Otherwise,
     * this is the offset from the start of the text.  If the parser
     * does not support this field, it will have a value < 0.
     * @stable ICU 2.0
     */
    int32_t        offset;

    /**
     * Textual context before the error.  Null-terminated.  The empty
     * string if not supported by parser.
     * @stable ICU 2.0   
     */
    UChar          preContext[U_PARSE_CONTEXT_LEN];

    /**
     * The error itself and/or textual context after the error.
     * Null-terminated.  The empty string if not supported by parser.
     * @stable ICU 2.0   
     */
    UChar          postContext[U_PARSE_CONTEXT_LEN];

} UParseError;

#endif
