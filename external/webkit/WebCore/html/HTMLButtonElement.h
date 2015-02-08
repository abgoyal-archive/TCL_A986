

#ifndef HTMLButtonElement_h
#define HTMLButtonElement_h

#include "HTMLFormControlElement.h"

namespace WebCore {

class HTMLButtonElement : public HTMLFormControlElement {
public:
    HTMLButtonElement(const QualifiedName&, Document*, HTMLFormElement* = 0);
    virtual ~HTMLButtonElement();

    virtual const AtomicString& formControlType() const;
        
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void defaultEventHandler(Event*);
    virtual bool appendFormData(FormDataList&, bool);

    virtual bool isEnumeratable() const { return true; } 

    virtual bool isSuccessfulSubmitButton() const;
    virtual bool isActivatedSubmit() const;
    virtual void setActivatedSubmit(bool flag);

    virtual void accessKeyAction(bool sendToAnyElement);

    virtual bool canStartSelection() const { return false; }

    String accessKey() const;
    void setAccessKey(const String&);

    String value() const;
    void setValue(const String&);

    virtual bool willValidate() const { return false; }
    
private:
    enum Type { SUBMIT, RESET, BUTTON };
    virtual bool isOptionalFormControl() const { return true; }

    Type m_type;
    bool m_activeSubmit;
};

} // namespace

#endif
