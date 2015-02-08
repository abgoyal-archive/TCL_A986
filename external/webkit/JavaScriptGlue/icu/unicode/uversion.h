

/*===========================================================================*/
/* Main ICU version information                                              */
/*===========================================================================*/

#ifndef UVERSION_H
#define UVERSION_H

/** IMPORTANT: When updating version, the following things need to be done:   */
/** source/common/unicode/uversion.h - this file: update major, minor,        */
/**        patchlevel, suffix, version, short version constants, namespace,   */
/**                                                             and copyright */
/** source/common/common.dsp - update 'Output file name' on the link tab so   */
/**                   that it contains the new major/minor combination        */
/** source/i18n/i18n.dsp - same as for the common.dsp                         */
/** source/layout/layout.dsp - same as for the common.dsp                     */
/** source/stubdata/stubdata.dsp - same as for the common.dsp                 */
/** source/extra/ustdio/ustdio.dsp - same as for the common.dsp               */
/** source/data/makedata.mak - change U_ICUDATA_NAME so that it contains      */
/**                            the new major/minor combination                */
/** source/tools/genren/genren.pl - use this script according to the README   */
/**                    in that folder                                         */

#include "unicode/umachine.h"

#define U_COPYRIGHT_STRING \
  " Copyright (C) 2004, International Business Machines Corporation and others. All Rights Reserved. "

#define U_COPYRIGHT_STRING_LENGTH  128

#define U_ICU_VERSION_MAJOR_NUM 3

#define U_ICU_VERSION_MINOR_NUM 2

#define U_ICU_VERSION_PATCHLEVEL_NUM 0

#define U_ICU_VERSION_SUFFIX _3_2

#define U_ICU_VERSION "3.2"

#define U_ICU_VERSION_SHORT "32"

#define U_MAX_VERSION_LENGTH 4

#define U_VERSION_DELIMITER '.'

#define U_MAX_VERSION_STRING_LENGTH 20

typedef uint8_t UVersionInfo[U_MAX_VERSION_LENGTH];

#if U_HAVE_NAMESPACE && defined(XP_CPLUSPLUS)
#if U_DISABLE_RENAMING
#define U_ICU_NAMESPACE icu
namespace U_ICU_NAMESPACE { }
#else
#define U_ICU_NAMESPACE icu_3_2
namespace U_ICU_NAMESPACE { }
namespace icu = U_ICU_NAMESPACE;
#endif
U_NAMESPACE_USE
#endif


/*===========================================================================*/
/* General version helper functions. Definitions in putil.c                  */
/*===========================================================================*/

U_STABLE void U_EXPORT2
u_versionFromString(UVersionInfo versionArray, const char *versionString);

U_STABLE void U_EXPORT2
u_versionToString(UVersionInfo versionArray, char *versionString);

U_STABLE void U_EXPORT2
u_getVersion(UVersionInfo versionArray);



#define UCOL_RUNTIME_VERSION 5

#define UCOL_BUILDER_VERSION 6

/*#define UCOL_FRACTIONAL_UCA_VERSION 4*/

#define UCOL_TAILORINGS_VERSION 1

#endif
