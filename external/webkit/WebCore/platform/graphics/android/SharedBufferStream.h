

#ifndef WebCore_SharedBufferStream_DEFINED
#define WebCore_SharedBufferStream_DEFINED

#include "SkStream.h"
#include "SharedBuffer.h"

namespace WebCore {

    /** Subclass of SkStream that wrapps a webcore SharedBuffer object. To
        allow this object to be deleted from any thread, the impl will ensure
        that we unref the SharedBuffer object from the correct webcore thread.
     */
    class SharedBufferStream : public SkMemoryStream {
    public:
        SharedBufferStream(SharedBuffer* buffer);
        virtual ~SharedBufferStream();

    private:
        // don't allow this to change our data. should not get called, but we
        // override here just to be sure
        virtual void setMemory(const void* data, size_t length, bool copyData) {
            sk_throw();
        }

        // we share ownership of this with webkit
        SharedBuffer* fBuffer;
    };                           
}

#endif
