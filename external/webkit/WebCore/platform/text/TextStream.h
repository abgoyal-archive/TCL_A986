

#ifndef TextStream_h
#define TextStream_h

#include <wtf/Vector.h>
#include <wtf/unicode/Unicode.h>

namespace WebCore {

class String;

class TextStream {
public:
    TextStream& operator<<(bool);
    TextStream& operator<<(int);
    TextStream& operator<<(unsigned);
    TextStream& operator<<(long);
    TextStream& operator<<(unsigned long);
    TextStream& operator<<(float);
    TextStream& operator<<(double);
    TextStream& operator<<(const char*);
    TextStream& operator<<(void*);
    TextStream& operator<<(const String&);
#if OS(WINDOWS) && PLATFORM(X86_64) && COMPILER(MSVC)
    TextStream& operator<<(unsigned __int64);
    TextStream& operator<<(__int64);
#endif

    String release();

private:
    Vector<UChar> m_text;
};

}

#endif
