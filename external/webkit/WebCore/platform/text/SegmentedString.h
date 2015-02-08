

#ifndef SegmentedString_h
#define SegmentedString_h

#include "PlatformString.h"
#include <wtf/Deque.h>

namespace WebCore {

class SegmentedString;

class SegmentedSubstring {
public:
    SegmentedSubstring() : m_length(0), m_current(0), m_doNotExcludeLineNumbers(true) {}
    SegmentedSubstring(const String& str)
        : m_length(str.length())
        , m_current(str.isEmpty() ? 0 : str.characters())
        , m_string(str)
        , m_doNotExcludeLineNumbers(true)
    {
    }

    SegmentedSubstring(const UChar* str, int length) : m_length(length), m_current(length == 0 ? 0 : str), m_doNotExcludeLineNumbers(true) {}

    void clear() { m_length = 0; m_current = 0; }
    
    bool excludeLineNumbers() const { return !m_doNotExcludeLineNumbers; }
    bool doNotExcludeLineNumbers() const { return m_doNotExcludeLineNumbers; }

    void setExcludeLineNumbers() { m_doNotExcludeLineNumbers = false; }

    void appendTo(String& str) const
    {
        if (m_string.characters() == m_current) {
            if (str.isEmpty())
                str = m_string;
            else
                str.append(m_string);
        } else {
            str.append(String(m_current, m_length));
        }
    }

public:
    int m_length;
    const UChar* m_current;

private:
    String m_string;
    bool m_doNotExcludeLineNumbers;
};

class SegmentedString {
public:
    SegmentedString()
        : m_pushedChar1(0), m_pushedChar2(0), m_currentChar(0), m_composite(false) {}
    SegmentedString(const UChar* str, int length) : m_pushedChar1(0), m_pushedChar2(0)
        , m_currentString(str, length), m_currentChar(m_currentString.m_current), m_composite(false) {}
    SegmentedString(const String& str)
        : m_pushedChar1(0), m_pushedChar2(0), m_currentString(str)
        , m_currentChar(m_currentString.m_current), m_composite(false) {}
    SegmentedString(const SegmentedString&);

    const SegmentedString& operator=(const SegmentedString&);

    void clear();

    void append(const SegmentedString&);
    void prepend(const SegmentedString&);
    
    bool excludeLineNumbers() const { return m_currentString.excludeLineNumbers(); }
    void setExcludeLineNumbers();

    void push(UChar c)
    {
        if (!m_pushedChar1) {
            m_pushedChar1 = c;
            m_currentChar = m_pushedChar1 ? &m_pushedChar1 : m_currentString.m_current;
        } else {
            ASSERT(!m_pushedChar2);
            m_pushedChar2 = c;
        }
    }
    
    bool isEmpty() const { return !current(); }
    unsigned length() const;

    void advance()
    {
        if (!m_pushedChar1 && m_currentString.m_length > 1) {
            --m_currentString.m_length;
            m_currentChar = ++m_currentString.m_current;
            return;
        }
        advanceSlowCase();
    }
    
    void advancePastNewline(int& lineNumber)
    {
        ASSERT(*current() == '\n');
        if (!m_pushedChar1 && m_currentString.m_length > 1) {
            lineNumber += m_currentString.doNotExcludeLineNumbers();
            --m_currentString.m_length;
            m_currentChar = ++m_currentString.m_current;
            return;
        }
        advanceSlowCase(lineNumber);
    }
    
    void advancePastNonNewline()
    {
        ASSERT(*current() != '\n');
        if (!m_pushedChar1 && m_currentString.m_length > 1) {
            --m_currentString.m_length;
            m_currentChar = ++m_currentString.m_current;
            return;
        }
        advanceSlowCase();
    }
    
    void advance(int& lineNumber)
    {
        if (!m_pushedChar1 && m_currentString.m_length > 1) {
            lineNumber += (*m_currentString.m_current == '\n') & m_currentString.doNotExcludeLineNumbers();
            --m_currentString.m_length;
            m_currentChar = ++m_currentString.m_current;
            return;
        }
        advanceSlowCase(lineNumber);
    }
    
    bool escaped() const { return m_pushedChar1; }
    
    String toString() const;

    const UChar& operator*() const { return *current(); }
    const UChar* operator->() const { return current(); }
    
private:
    void append(const SegmentedSubstring&);
    void prepend(const SegmentedSubstring&);

    void advanceSlowCase();
    void advanceSlowCase(int& lineNumber);
    void advanceSubstring();
    const UChar* current() const { return m_currentChar; }

    UChar m_pushedChar1;
    UChar m_pushedChar2;
    SegmentedSubstring m_currentString;
    const UChar* m_currentChar;
    Deque<SegmentedSubstring> m_substrings;
    bool m_composite;
};

}

#endif
