

#ifndef HTMLMapElement_h
#define HTMLMapElement_h

#include "HTMLElement.h"

namespace WebCore {

class IntSize;
class HitTestResult;
class HTMLImageElement;
    
class HTMLMapElement : public HTMLElement {
public:
    HTMLMapElement(const QualifiedName&, Document*);
    ~HTMLMapElement();

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 1; }
    virtual bool checkDTD(const Node*);

    const AtomicString& getName() const { return m_name; }

    virtual void parseMappedAttribute(MappedAttribute*);

    bool mapMouseEvent(int x, int y, const IntSize&, HitTestResult&);
    
    HTMLImageElement* imageElement() const;
    PassRefPtr<HTMLCollection> areas();

    String name() const;
    void setName(const String&);

private:
    AtomicString m_name;
};

} //namespace

#endif
