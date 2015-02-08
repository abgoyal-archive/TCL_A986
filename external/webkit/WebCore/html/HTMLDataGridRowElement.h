

#ifndef HTMLDataGridRowElement_h
#define HTMLDataGridRowElement_h

#if ENABLE(DATAGRID)

#include "HTMLElement.h"

namespace WebCore {

class HTMLDataGridRowElement : public HTMLElement {
public:
    HTMLDataGridRowElement(const QualifiedName&, Document*);

    virtual int tagPriority() const { return 2; } // Same as <option>s.
    virtual bool checkDTD(const Node*);

    bool selected() const;
    void setSelected(bool);
    
    bool focused() const;
    void setFocused(bool);
    
    bool expanded() const;
    void setExpanded(bool);
};

} // namespace WebCore

#endif

#endif // HTMLDataGridRowElement_h
