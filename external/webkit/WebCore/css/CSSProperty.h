

#ifndef CSSProperty_h
#define CSSProperty_h

#include "CSSValue.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class CSSProperty : public FastAllocBase {
public:
    CSSProperty(int propID, PassRefPtr<CSSValue> value, bool important = false, int shorthandID = 0, bool implicit = false)
        : m_id(propID)
        , m_shorthandID(shorthandID)
        , m_important(important)
        , m_implicit(implicit)
        , m_value(value)
    {
    }

    CSSProperty& operator=(const CSSProperty& other)
    {
        m_id = other.m_id;
        m_shorthandID = other.m_shorthandID;
        m_important = other.m_important;
        m_implicit = other.m_implicit;
        m_value = other.m_value;
        return *this;
    }

    int id() const { return m_id; }
    int shorthandID() const { return m_shorthandID; }

    bool isImportant() const { return m_important; }
    bool isImplicit() const { return m_implicit; }

    CSSValue* value() const { return m_value.get(); }
    
    String cssText() const;

    friend bool operator==(const CSSProperty&, const CSSProperty&);

    // Make sure the following fits in 4 bytes. Really.
    int m_id : 15;
    int m_shorthandID : 15; // If this property was set as part of a shorthand, gives the shorthand.
    bool m_important : 1;
    bool m_implicit : 1; // Whether or not the property was set implicitly as the result of a shorthand.

    RefPtr<CSSValue> m_value;
};

} // namespace WebCore

namespace WTF {
    // Properties in Vector can be initialized with memset and moved using memcpy.
    template<> struct VectorTraits<WebCore::CSSProperty> : SimpleClassVectorTraits { };
}

#endif // CSSProperty_h
