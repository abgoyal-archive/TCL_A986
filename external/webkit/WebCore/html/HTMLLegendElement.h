

#ifndef HTMLLegendElement_h
#define HTMLLegendElement_h

#include "HTMLFormControlElement.h"

namespace WebCore {

class HTMLLegendElement : public HTMLFormControlElement {
public:
    HTMLLegendElement(const QualifiedName&, Document*, HTMLFormElement* = 0);
    virtual ~HTMLLegendElement();

    virtual bool supportsFocus() const;
    virtual const AtomicString& formControlType() const;
    virtual void accessKeyAction(bool sendToAnyElement);

    /**
     * The first form element in the legend's fieldset 
     */
    Element* formElement();

    String accessKey() const;
    void setAccessKey(const String &);

    String align() const;
    void setAlign(const String &);
    
    void focus(bool restorePreviousSelection = true);
};

} //namespace

#endif
