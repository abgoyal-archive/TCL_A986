

#ifndef HTMLLabelElement_h
#define HTMLLabelElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLLabelElement : public HTMLElement {
public:
    HTMLLabelElement(const QualifiedName&, Document*);
    virtual ~HTMLLabelElement();

    virtual int tagPriority() const { return 5; }

    virtual bool isFocusable() const;

    virtual void accessKeyAction(bool sendToAnyElement);

    // Overridden to update the hover/active state of the corresponding control.
    virtual void setActive(bool = true, bool pause = false);
    virtual void setHovered(bool = true);

    // Overridden to either click() or focus() the corresponding control.
    virtual void defaultEventHandler(Event*);

    HTMLElement* correspondingControl();

    String accessKey() const;
    void setAccessKey(const String&);

    String htmlFor() const;
    void setHtmlFor(const String&);

    void focus(bool restorePreviousSelection = true);

 private:
    String m_formElementID;
};

} //namespace

#endif
