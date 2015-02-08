

#include "config.h"
#include "ByteArray.h"

namespace WTF {

PassRefPtr<ByteArray> ByteArray::create(size_t size)
{
    unsigned char* buffer = new unsigned char[size + sizeof(ByteArray) - sizeof(size_t)];
    ASSERT((reinterpret_cast<size_t>(buffer) & 3) == 0);
    return adoptRef(new (buffer) ByteArray(size));
}

}
