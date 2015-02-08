
#ifndef _UWMSG
#define _UWMSG

#include <stdio.h>

#include "unicode/ures.h"

U_CFUNC UResourceBundle *u_wmsg_setPath(const char *path, UErrorCode *err);

/* Format a message and print it's output to a given file stream */
U_CFUNC int u_wmsg(FILE *fp, const char *tag, ... );

/* format an error message */
U_CFUNC const UChar* u_wmsg_errorName(UErrorCode err);

#endif
