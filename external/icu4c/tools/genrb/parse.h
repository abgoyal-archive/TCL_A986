
#ifndef PARSE_H
#define PARSE_H 1

#include "unicode/utypes.h"
#include "filestrm.h"
#include "ucbuf.h"

U_CDECL_BEGIN
/* One time parser initalisation */
void initParser(UBool omitBinaryCollation, UBool omitCollationRules);

/* Parse a ResourceBundle text file */
struct SRBRoot* parse(UCHARBUF *buf, const char* inputDir, const char* outputDir, UErrorCode *status);

U_CDECL_END

#endif
