
#include "unicode/utypes.h"
#include "unicode/errorcode.h"

U_NAMESPACE_BEGIN

UErrorCode ErrorCode::reset() {
    UErrorCode code = errorCode;
    errorCode = U_ZERO_ERROR;
    return code;
}

void ErrorCode::assertSuccess() const {
    if(isFailure()) {
        handleFailure();
    }
}

const char* ErrorCode::errorName() const {
  return u_errorName(errorCode);
}

U_NAMESPACE_END
