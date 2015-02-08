

#ifndef PossiblyNull_h
#define PossiblyNull_h

#include "Assertions.h"

namespace WTF {

template <typename T> struct PossiblyNull {
    PossiblyNull(T data)
        : m_data(data)
    {
    }
    PossiblyNull(const PossiblyNull<T>& source)
        : m_data(source.m_data)
    {
        source.m_data = 0;
    }
    ~PossiblyNull() { ASSERT(!m_data); }
    bool getValue(T& out) WARN_UNUSED_RETURN;
private:
    mutable T m_data;
};

template <typename T> bool PossiblyNull<T>::getValue(T& out)
{
    out = m_data;
    bool result = !!m_data;
    m_data = 0;
    return result;
}

}

#endif
