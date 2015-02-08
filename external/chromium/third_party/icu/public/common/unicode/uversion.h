
/*===========================================================================*/
/* Main ICU version information                                              */
/*===========================================================================*/

#ifndef UVERSION_H
#define UVERSION_H


#include "unicode/umachine.h"

#define U_COPYRIGHT_STRING \
  " Copyright (C) 2009, International Business Machines Corporation and others. All Rights Reserved. "

#define U_COPYRIGHT_STRING_LENGTH  128

#define U_ICU_VERSION_MAJOR_NUM 4

#define U_ICU_VERSION_MINOR_NUM 2

#define U_ICU_VERSION_PATCHLEVEL_NUM 1

#ifndef U_ICU_VERSION_BUILDLEVEL_NUM
#define U_ICU_VERSION_BUILDLEVEL_NUM 0
#endif

#define U_ICU_VERSION_SUFFIX _4_2

#define U_ICU_VERSION "4.2.1"

#define U_ICU_VERSION_SHORT "42"

#define U_MAX_VERSION_LENGTH 4

#define U_VERSION_DELIMITER '.'

#define U_MAX_VERSION_STRING_LENGTH 20

typedef uint8_t UVersionInfo[U_MAX_VERSION_LENGTH];

/*===========================================================================*/
/* C++ namespace if supported. Versioned unless versioning is disabled.      */
/*===========================================================================*/





/* Define namespace symbols if the compiler supports it. */
#if U_HAVE_NAMESPACE && defined(XP_CPLUSPLUS)
#   if U_DISABLE_RENAMING
#       define U_ICU_NAMESPACE icu
        namespace U_ICU_NAMESPACE { }
#   else
#       define U_ICU_NAMESPACE icu_4_2
        namespace U_ICU_NAMESPACE { }
        namespace icu = U_ICU_NAMESPACE;
#   endif

#   define U_NAMESPACE_BEGIN namespace U_ICU_NAMESPACE {
#   define U_NAMESPACE_END  }
#   define U_NAMESPACE_USE using namespace U_ICU_NAMESPACE;
#   define U_NAMESPACE_QUALIFIER U_ICU_NAMESPACE::

#   ifndef U_USING_ICU_NAMESPACE
        /* Chrome-specific: Set to 0 to force namespace
           declarations in Chrome ICU usage. */
#       define U_USING_ICU_NAMESPACE 0
#   endif
#   if U_USING_ICU_NAMESPACE
        U_NAMESPACE_USE
#   endif
#else
#   define U_NAMESPACE_BEGIN
#   define U_NAMESPACE_END
#   define U_NAMESPACE_USE
#   define U_NAMESPACE_QUALIFIER
#endif


/*===========================================================================*/
/* General version helper functions. Definitions in putil.c                  */
/*===========================================================================*/

U_STABLE void U_EXPORT2
u_versionFromString(UVersionInfo versionArray, const char *versionString);

U_STABLE void U_EXPORT2
u_versionFromUString(UVersionInfo versionArray, const UChar *versionString);

U_STABLE void U_EXPORT2
u_versionToString(UVersionInfo versionArray, char *versionString);

U_STABLE void U_EXPORT2
u_getVersion(UVersionInfo versionArray);

U_STABLE int32_t U_EXPORT2
u_compareVersions(UVersionInfo v1, UVersionInfo v2);



#define UCOL_RUNTIME_VERSION 6

#define UCOL_BUILDER_VERSION 7

#define UCOL_TAILORINGS_VERSION 1

#endif
