

#include "config.h"
#include "TextEncodingDetector.h"

#include "TextEncoding.h"
#include <wtf/UnusedParam.h>

#ifndef BUILDING_ON_TIGER
#include "unicode/ucnv.h"
#include "unicode/ucsdet.h"
#endif

namespace WebCore {

bool detectTextEncoding(const char* data, size_t len,
                        const char* hintEncodingName,
                        TextEncoding* detectedEncoding)
{
    *detectedEncoding = TextEncoding();
#ifdef BUILDING_ON_TIGER
    // Tiger came with ICU 3.2 and does not have the encoding detector.
    UNUSED_PARAM(data);
    UNUSED_PARAM(len);
    UNUSED_PARAM(hintEncodingName);
    return false;
#else
    int matchesCount = 0; 
    UErrorCode status = U_ZERO_ERROR;
    UCharsetDetector* detector = ucsdet_open(&status);
    if (U_FAILURE(status))
        return false;
    ucsdet_enableInputFilter(detector, true);
    ucsdet_setText(detector, data, static_cast<int32_t>(len), &status); 
    if (U_FAILURE(status))
        return false;

    // FIXME: A few things we can do other than improving
    // the ICU detector itself. 
    // 1. Use ucsdet_detectAll and pick the most likely one given
    // "the context" (parent-encoding, referrer encoding, etc).
    // 2. 'Emulate' Firefox/IE's non-Universal detectors (e.g.
    // Chinese, Japanese, Russian, Korean and Hebrew) by picking the 
    // encoding with a highest confidence among the detector-specific
    // limited set of candidate encodings.
    // Below is a partial implementation of the first part of what's outlined
    // above.
    const UCharsetMatch** matches = ucsdet_detectAll(detector, &matchesCount, &status);
    if (U_FAILURE(status)) {
        ucsdet_close(detector);
        return false;
    }

    const char* encoding = 0;
    if (hintEncodingName) {
        TextEncoding hintEncoding(hintEncodingName);
        // 10 is the minimum confidence value consistent with the codepoint
        // allocation in a given encoding. The size of a chunk passed to
        // us varies even for the same html file (apparently depending on 
        // the network load). When we're given a rather short chunk, we 
        // don't have a sufficiently reliable signal other than the fact that
        // the chunk is consistent with a set of encodings. So, instead of
        // setting an arbitrary threshold, we have to scan all the encodings
        // consistent with the data.  
        const int32_t kThresold = 10;
        for (int i = 0; i < matchesCount; ++i) {
            int32_t confidence = ucsdet_getConfidence(matches[i], &status);
            if (U_FAILURE(status)) {
                status = U_ZERO_ERROR;
                continue;
            }
            if (confidence < kThresold)
                break;
            const char* matchEncoding = ucsdet_getName(matches[i], &status);
            if (U_FAILURE(status)) {
                status = U_ZERO_ERROR;
                continue;
            }
            if (TextEncoding(matchEncoding) == hintEncoding) {
                encoding = hintEncodingName;
                break;
            }
        }
    }
    // If no match is found so far, just pick the top match. 
    // This can happen, say, when a parent frame in EUC-JP refers to
    // a child frame in Shift_JIS and both frames do NOT specify the encoding
    // making us resort to auto-detection (when it IS turned on).
    if (!encoding && matchesCount > 0)
        encoding = ucsdet_getName(matches[0], &status);
    if (U_SUCCESS(status)) {
        *detectedEncoding = TextEncoding(encoding);
        ucsdet_close(detector);
        return true;
    }    
    ucsdet_close(detector);
    return false;
#endif
}

}
