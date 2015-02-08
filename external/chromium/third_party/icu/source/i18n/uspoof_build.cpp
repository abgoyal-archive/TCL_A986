
#include "unicode/utypes.h"
#include "unicode/uspoof.h"
#include "unicode/unorm.h"
#include "unicode/uregex.h"
#include "unicode/ustring.h"
#include "cmemory.h"
#include "uspoof_impl.h"
#include "uhash.h"
#include "uvector.h"
#include "uassert.h"
#include "uarrsort.h"
#include "uspoof_buildconf.h"
#include "uspoof_buildwsconf.h"

#if !UCONFIG_NO_NORMALIZATION

U_NAMESPACE_USE


// The main data building function

U_CAPI USpoofChecker * U_EXPORT2
uspoof_openFromSource(const char *confusables,  int32_t confusablesLen,
                      const char *confusablesWholeScript, int32_t confusablesWholeScriptLen,
                      int32_t *errorType, UParseError *pe, UErrorCode *status) {

    if (U_FAILURE(*status)) {
        return NULL;
    }
#if UCONFIG_NO_REGULAR_EXPRESSIONS 
    *status = U_UNSUPPORTED_ERROR;      
    return NULL;
#else
    if (errorType!=NULL) {
        *errorType = 0;
    }
    if (pe != NULL) {
        pe->line = 0;
        pe->offset = 0;
        pe->preContext[0] = 0;
        pe->postContext[0] = 0;
    }

    // Set up a shell of a spoof detector, with empty data.
    SpoofData *newSpoofData = new SpoofData(*status);
    SpoofImpl *This = new SpoofImpl(newSpoofData, *status);

    // Compile the binary data from the source (text) format.
    ConfusabledataBuilder::buildConfusableData(This, confusables, confusablesLen, errorType, pe, *status);
    buildWSConfusableData(This, confusablesWholeScript, confusablesWholeScriptLen, pe, *status);
    
    if (U_FAILURE(*status)) {
        delete This;
        This = NULL;
    }
    return (USpoofChecker *)This;
#endif // UCONFIG_NO_REGULAR_EXPRESSIONS 
}

#endif
