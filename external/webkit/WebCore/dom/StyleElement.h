
#ifndef StyleElement_h
#define StyleElement_h

#include "CSSStyleSheet.h"

namespace WebCore {

class Element;

class StyleElement {
public:
    StyleElement();
    virtual ~StyleElement() {}

protected:
    StyleSheet* sheet(Element*);

    virtual void setLoading(bool) {}

    virtual const AtomicString& type() const = 0;
    virtual const AtomicString& media() const = 0;

    void insertedIntoDocument(Document*, Element*);
    void removedFromDocument(Document*);
    void process(Element*);

    void createSheet(Element* e, const String& text = String());

protected:
    RefPtr<CSSStyleSheet> m_sheet;
};

} //namespace

#endif
