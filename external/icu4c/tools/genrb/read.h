
#ifndef READ_H
#define READ_H 1

#include "unicode/utypes.h"
#include "ustr.h"
#include "ucbuf.h"

enum ETokenType
{
    TOK_STRING,          /* A string token, such as "MonthNames" */
    TOK_OPEN_BRACE,      /* An opening brace character */
    TOK_CLOSE_BRACE,     /* A closing brace character */
    TOK_COMMA,           /* A comma */
    TOK_COLON,           /* A colon */

    TOK_EOF,             /* End of the file has been reached successfully */
    TOK_ERROR,           /* An error, such an unterminated quoted string */
    TOK_TOKEN_COUNT      /* Number of "real" token types */
};

UChar32 unescape(UCHARBUF *buf, UErrorCode *status);

void resetLineNumber(void);

enum ETokenType getNextToken(UCHARBUF *buf,
                 struct UString *token,
                 uint32_t *linenumber, /* out: linenumber of token */
                 struct UString *comment,
                 UErrorCode *status);

#endif
