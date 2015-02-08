
#ifndef UCADATA_H
#define UCADATA_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

#include "ucol_elm.h"
#include <stdio.h>
#include <string.h>
#include "unicode/utypes.h"
#include "unicode/uchar.h"
#include "ucol_imp.h"
#include "uhash.h"
#include "unewdata.h"


void deleteElement(void *element);
int32_t readElement(char **from, char *to, char separator, UErrorCode *status);
uint32_t getSingleCEValue(char *primary, char *secondary, char *tertiary, UBool caseBit, UErrorCode *status);
void printOutTable(UCATableHeader *myData, UErrorCode *status);
UCAElements *readAnElement(FILE *data, tempUCATable *t, UCAConstants *consts, UErrorCode *status);

#endif /* #if !UCONFIG_NO_COLLATION */

#endif
