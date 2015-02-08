

#ifndef CSSValueList_h
#define CSSValueList_h

#include "CSSValue.h"
#include <wtf/PassRefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class CSSParserValueList;

class CSSValueList : public CSSValue {
public:
    static PassRefPtr<CSSValueList> createCommaSeparated()
    {
        return adoptRef(new CSSValueList(false));
    }
    static PassRefPtr<CSSValueList> createSpaceSeparated()
    {
        return adoptRef(new CSSValueList(true));
    }
    static PassRefPtr<CSSValueList> createFromParserValueList(CSSParserValueList* list)
    {
        return adoptRef(new CSSValueList(list));
    }

    virtual ~CSSValueList();

    size_t length() const { return m_values.size(); }
    CSSValue* item(unsigned);
    CSSValue* itemWithoutBoundsCheck(unsigned index) { return m_values[index].get(); }

    void append(PassRefPtr<CSSValue>);
    void prepend(PassRefPtr<CSSValue>);
    bool removeAll(CSSValue*);
    bool hasValue(CSSValue*);
    PassRefPtr<CSSValueList> copy();

    virtual String cssText() const;

    CSSParserValueList* createParserValueList() const;

    virtual void addSubresourceStyleURLs(ListHashSet<KURL>&, const CSSStyleSheet*);

protected:
    CSSValueList(bool isSpaceSeparated);
    CSSValueList(CSSParserValueList*);

private:
    virtual bool isValueList() const { return true; }

    virtual unsigned short cssValueType() const;

    Vector<RefPtr<CSSValue> > m_values;
    bool m_isSpaceSeparated;
};

} // namespace WebCore

#endif // CSSValueList_h
