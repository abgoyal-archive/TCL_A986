

#ifndef HTMLDataGridCellElement_h
#define HTMLDataGridCellElement_h

#if ENABLE(DATAGRID)

#include "HTMLElement.h"

namespace WebCore {

class HTMLDataGridCellElement : public HTMLElement {
public:
    HTMLDataGridCellElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }
    
    String label() const;
    void setLabel(const String&);
    
    bool focused() const;
    void setFocused(bool);

    bool checked() const;
    void setChecked(bool);
    
    bool indeterminate() const;
    void setIndeterminate(bool);
    
    float progress() const;
    void setProgress(float);
};

} // namespace WebCore

#endif

#endif // HTMLDataGridCellElement_h
