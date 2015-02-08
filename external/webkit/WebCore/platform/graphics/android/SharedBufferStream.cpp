

#include "config.h"
#include "JavaSharedClient.h"
#include "SharedBuffer.h"
#include "SharedBufferStream.h"

using namespace android;

namespace WebCore {

    static void CallDeref(void* buffer) {
        ((SharedBuffer*)buffer)->deref();
    }

    SharedBufferStream::SharedBufferStream(SharedBuffer* buffer)
            : SkMemoryStream(buffer->data(), buffer->size(), false) {
        fBuffer = buffer;
        buffer->ref();
    }

    SharedBufferStream::~SharedBufferStream() {
        // we can't necessarily call fBuffer->deref() here, as we may be
        // in a different thread from webkit, and SharedBuffer is not
        // threadsafe. Therefore we defer it until it can be executed in the
        // webkit thread.
        JavaSharedClient::EnqueueFunctionPtr(CallDeref, fBuffer);
    }

}
