

#include "config.h"
#include "UStringImpl.h"

#include "Identifier.h"
#include "UString.h"
#include <wtf/unicode/UTF8.h>

using namespace WTF::Unicode;
using namespace std;

namespace JSC {

PassRefPtr<UStringImpl> UStringImpl::create(const char* c)
{
    ASSERT(c);

    if (!c[0])
        return &UStringImpl::empty();

    size_t length = strlen(c);
    UChar* d;
    PassRefPtr<UStringImpl> result = UStringImpl::createUninitialized(length, d);
    for (size_t i = 0; i < length; i++)
        d[i] = static_cast<unsigned char>(c[i]); // use unsigned char to zero-extend instead of sign-extend
    return result;
}

PassRefPtr<UStringImpl> UStringImpl::create(const char* c, int length)
{
    ASSERT(c);

    if (!length)
        return &UStringImpl::empty();

    UChar* d;
    PassRefPtr<UStringImpl> result = UStringImpl::createUninitialized(length, d);
    for (int i = 0; i < length; i++)
        d[i] = static_cast<unsigned char>(c[i]); // use unsigned char to zero-extend instead of sign-extend
    return result;
}

PassRefPtr<UStringImpl> UStringImpl::create(const UChar* buffer, int length)
{
    UChar* newBuffer;
    PassRefPtr<UStringImpl> impl = createUninitialized(length, newBuffer);
    copyChars(newBuffer, buffer, length);
    return impl;
}

SharedUChar* UStringImpl::baseSharedBuffer()
{
    ASSERT((bufferOwnership() == BufferShared)
        || ((bufferOwnership() == BufferOwned) && !m_dataBuffer.asPtr<void*>()));

    if (bufferOwnership() != BufferShared)
        m_dataBuffer = UntypedPtrAndBitfield(SharedUChar::create(new OwnFastMallocPtr<UChar>(m_data)).releaseRef(), BufferShared);

    return m_dataBuffer.asPtr<SharedUChar*>();
}

SharedUChar* UStringImpl::sharedBuffer()
{
    if (m_length < s_minLengthToShare)
        return 0;
    ASSERT(!isStatic());

    UStringImpl* owner = bufferOwnerString();
    if (owner->bufferOwnership() == BufferInternal)
        return 0;

    return owner->baseSharedBuffer();
}

UStringImpl::~UStringImpl()
{
    ASSERT(!isStatic());
    checkConsistency();

    if (isIdentifier())
        Identifier::remove(this);

    if (bufferOwnership() != BufferInternal) {
        if (bufferOwnership() == BufferOwned)
            fastFree(m_data);
        else if (bufferOwnership() == BufferSubstring)
            m_dataBuffer.asPtr<UStringImpl*>()->deref();
        else {
            ASSERT(bufferOwnership() == BufferShared);
            m_dataBuffer.asPtr<SharedUChar*>()->deref();
        }
    }
}

}
