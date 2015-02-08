

#ifndef CSSPropertyLonghand_h
#define CSSPropertyLonghand_h

namespace WebCore {

class CSSPropertyLonghand {
public:
    CSSPropertyLonghand()
        : m_properties(0)
        , m_length(0)
    {
    }

    CSSPropertyLonghand(const int* firstProperty, unsigned numProperties)
        : m_properties(firstProperty)
        , m_length(numProperties)
    {
    }

    const int* properties() const { return m_properties; }
    unsigned length() const { return m_length; }

private:
    const int* m_properties;
    unsigned m_length;
};

// Returns an empty list if the property is not a shorthand
CSSPropertyLonghand longhandForProperty(int);

} // namespace WebCore

#endif // CSSPropertyLonghand_h
