

#include "config.h"
#include "FormDataList.h"

namespace WebCore {

FormDataList::FormDataList(const TextEncoding& c)
    : m_encoding(c)
{
}

void FormDataList::appendString(const CString& s)
{
    m_list.append(s);
}

// Change plain CR and plain LF to CRLF pairs.
static CString fixLineBreaks(const CString& s)
{
    // Compute the length.
    unsigned newLen = 0;
    const char* p = s.data();
    while (char c = *p++) {
        if (c == '\r') {
            // Safe to look ahead because of trailing '\0'.
            if (*p != '\n') {
                // Turn CR into CRLF.
                newLen += 2;
            }
        } else if (c == '\n') {
            // Turn LF into CRLF.
            newLen += 2;
        } else {
            // Leave other characters alone.
            newLen += 1;
        }
    }
    if (newLen == s.length()) {
        return s;
    }
    
    // Make a copy of the string.
    p = s.data();
    char* q;
    CString result = CString::newUninitialized(newLen, q);
    while (char c = *p++) {
        if (c == '\r') {
            // Safe to look ahead because of trailing '\0'.
            if (*p != '\n') {
                // Turn CR into CRLF.
                *q++ = '\r';
                *q++ = '\n';
            }
        } else if (c == '\n') {
            // Turn LF into CRLF.
            *q++ = '\r';
            *q++ = '\n';
        } else {
            // Leave other characters alone.
            *q++ = c;
        }
    }
    return result;
}

void FormDataList::appendString(const String& s)
{
    CString cstr = fixLineBreaks(m_encoding.encode(s.characters(), s.length(), EntitiesForUnencodables));
    m_list.append(cstr);
}

} // namespace
