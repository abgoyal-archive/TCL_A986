

#ifndef StringBuilder_h
#define StringBuilder_h

#include <wtf/Vector.h>

namespace JSC {

class StringBuilder {
public:
    void append(const UChar u)
    {
        buffer.append(u);
    }

    void append(const char* str)
    {
        append(str, strlen(str));
    }

    void append(const char* str, size_t len)
    {
        buffer.reserveCapacity(buffer.size() + len);
        for (size_t i = 0; i < len; i++)
            buffer.append(static_cast<unsigned char>(str[i]));
    }

    void append(const UChar* str, size_t len)
    {
        buffer.append(str, len);
    }

    void append(const UString& str)
    {
        buffer.append(str.data(), str.size());
    }

    bool isEmpty() { return buffer.isEmpty(); }
    void reserveCapacity(size_t newCapacity) { buffer.reserveCapacity(newCapacity); }
    void resize(size_t size) { buffer.resize(size); }
    size_t size() const { return buffer.size(); }

    UChar operator[](size_t i) const { return buffer.at(i); }

    UString build()
    {
        buffer.shrinkToFit();
        if (buffer.size() && !buffer.data())
            CRASH();
        return UString::adopt(buffer);
    }

protected:
    Vector<UChar, 64> buffer;
};

}

#endif
