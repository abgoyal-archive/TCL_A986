
#ifndef __PLRUNS_H
#define __PLRUNS_H

#include "unicode/utypes.h"
#include "unicode/ubidi.h"
#include "layout/LETypes.h"

#include "layout/loengine.h"

typedef void pl_fontRuns;
typedef void pl_valueRuns;
typedef void pl_localeRuns;


U_INTERNAL pl_fontRuns * U_EXPORT2
pl_openFontRuns(const le_font **fonts,
                const le_int32 *limits,
                le_int32 count);

U_INTERNAL pl_fontRuns * U_EXPORT2
pl_openEmptyFontRuns(le_int32 initialCapacity);

U_INTERNAL void U_EXPORT2
pl_closeFontRuns(pl_fontRuns *fontRuns);

U_INTERNAL le_int32 U_EXPORT2
pl_getFontRunCount(const pl_fontRuns *fontRuns);

U_INTERNAL void U_EXPORT2
pl_resetFontRuns(pl_fontRuns *fontRuns);

U_INTERNAL le_int32 U_EXPORT2
pl_getFontRunLastLimit(const pl_fontRuns *fontRuns);

U_INTERNAL le_int32 U_EXPORT2
pl_getFontRunLimit(const pl_fontRuns *fontRuns,
                   le_int32 run);

U_INTERNAL const le_font * U_EXPORT2
pl_getFontRunFont(const pl_fontRuns *fontRuns,
                  le_int32 run);


U_INTERNAL le_int32 U_EXPORT2
pl_addFontRun(pl_fontRuns *fontRuns,
              const le_font *font,
              le_int32 limit);

U_INTERNAL pl_valueRuns * U_EXPORT2
pl_openValueRuns(const le_int32 *values,
                 const le_int32 *limits,
                 le_int32 count);

U_INTERNAL pl_valueRuns * U_EXPORT2
pl_openEmptyValueRuns(le_int32 initialCapacity);

U_INTERNAL void U_EXPORT2
pl_closeValueRuns(pl_valueRuns *valueRuns);

U_INTERNAL le_int32 U_EXPORT2
pl_getValueRunCount(const pl_valueRuns *valueRuns);

U_INTERNAL void U_EXPORT2
pl_resetValueRuns(pl_valueRuns *valueRuns);

U_INTERNAL le_int32 U_EXPORT2
pl_getValueRunLastLimit(const pl_valueRuns *valueRuns);

U_INTERNAL le_int32 U_EXPORT2
pl_getValueRunLimit(const pl_valueRuns *valueRuns,
                     le_int32 run);

U_INTERNAL le_int32 U_EXPORT2
pl_getValueRunValue(const pl_valueRuns *valueRuns,
                    le_int32 run);


U_INTERNAL le_int32 U_EXPORT2
pl_addValueRun(pl_valueRuns *valueRuns,
               le_int32 value,
               le_int32 limit);

U_INTERNAL pl_localeRuns * U_EXPORT2
pl_openLocaleRuns(const char **locales,
                  const le_int32 *limits,
                  le_int32 count);

U_INTERNAL pl_localeRuns * U_EXPORT2
pl_openEmptyLocaleRuns(le_int32 initialCapacity);

U_INTERNAL void U_EXPORT2
pl_closeLocaleRuns(pl_localeRuns *localeRuns);

U_INTERNAL le_int32 U_EXPORT2
pl_getLocaleRunCount(const pl_localeRuns *localeRuns);

U_INTERNAL void U_EXPORT2
pl_resetLocaleRuns(pl_localeRuns *localeRuns);

U_INTERNAL le_int32 U_EXPORT2
pl_getLocaleRunLastLimit(const pl_localeRuns *localeRuns);

U_INTERNAL le_int32 U_EXPORT2
pl_getLocaleRunLimit(const pl_localeRuns *localeRuns,
                     le_int32 run);

U_INTERNAL const char * U_EXPORT2
pl_getLocaleRunLocale(const pl_localeRuns *localeRuns,
                      le_int32 run);


U_INTERNAL le_int32 U_EXPORT2
pl_addLocaleRun(pl_localeRuns *localeRuns,
                const char *locale,
                le_int32 limit);

#endif
