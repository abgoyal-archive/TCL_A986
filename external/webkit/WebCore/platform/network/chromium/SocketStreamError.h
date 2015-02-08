

#ifndef SocketStreamError_h
#define SocketStreamError_h

#include "SocketStreamErrorBase.h"

namespace WebCore {

class SocketStreamError : public SocketStreamErrorBase {
public:
    SocketStreamError() { }
    explicit SocketStreamError(int errorCode)
            : SocketStreamErrorBase(errorCode) { }
};

}  // namespace WebCore

#endif  // SocketStreamError_h
