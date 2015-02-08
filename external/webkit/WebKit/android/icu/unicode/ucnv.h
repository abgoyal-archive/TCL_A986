

#ifndef ANDROID_UCNV_H
#define ANDROID_UCNV_H

// Include the real ucnv.h file from icu. Use a more exact reference so we do
// not conflict with this file.
#include <icu4c/common/unicode/ucnv.h>

namespace android {

U_STABLE UConverter* U_EXPORT2
ucnv_open_emoji(const char *converterName, UErrorCode *err);

}

// Redefine ucnv_open to android::ucnv_open_emoji. This relies heavily on the
// fact that this file will be included before any of the real icu headers.
// This is done in Android.mk by including WebKit/android/icu before the
// regular icu directory.
#undef ucnv_open
#define ucnv_open android::ucnv_open_emoji

#endif
