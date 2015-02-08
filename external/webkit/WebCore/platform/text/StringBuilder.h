

#ifndef StringBuilder_h
#define StringBuilder_h

#include "PlatformString.h"

namespace WebCore {

    class StringBuilder {
    public:
        StringBuilder() : m_totalLength(UINT_MAX) {}

        void setNonNull() { if (m_totalLength == UINT_MAX) m_totalLength = 0; }

        void append(const String&);
        void append(UChar);
        void append(char);
        
        void clear();
        unsigned length() const;

        String toString() const;

    private:
        bool isNull() const { return m_totalLength == UINT_MAX; }

        unsigned m_totalLength;
        Vector<String, 16> m_strings;
    };

}

#endif
