
#include "unicode/utypes.h"
#include "unicode/errorcode.h"

U_NAMESPACE_BEGIN

UErrorCode ErrorCode::reset() {
    UErrorCode code = errorCode;
    errorCode = U_ZERO_ERROR;
    return code;
}

void ErrorCode::check() const {
    if(isFailure()) {
        handleFailure();
    }
}

U_NAMESPACE_END
