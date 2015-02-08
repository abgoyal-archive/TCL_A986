

#ifndef WMLInputElement_h
#define WMLInputElement_h

#if ENABLE(WML)
#include "WMLFormControlElement.h"
#include "InputElement.h"

namespace WebCore {

class FormDataList;

class WMLInputElement : public WMLFormControlElement, public InputElement {
public:
    WMLInputElement(const QualifiedName& tagName, Document*);
    virtual ~WMLInputElement();

    virtual bool isKeyboardFocusable(KeyboardEvent*) const;
    virtual bool isMouseFocusable() const;
    virtual void dispatchFocusEvent();
    virtual void dispatchBlurEvent();
    virtual void updateFocusAppearance(bool restorePreviousSelection);
    virtual void aboutToUnload();

    virtual bool shouldUseInputMethod() const { return !m_isPasswordField; }
    virtual bool isChecked() const { return false; }
    virtual bool isAutofilled() const { return false; }
    virtual bool isIndeterminate() const { return false; }
    virtual bool isTextFormControl() const { return true; }
    virtual bool isRadioButton() const { return false; }
    virtual bool isTextField() const { return true; }
    virtual bool isSearchField() const { return false; }
    virtual bool isInputTypeHidden() const { return false; }
    virtual bool isPasswordField() const { return m_isPasswordField; }
    virtual bool searchEventsShouldBeDispatched() const { return false; }

    virtual int size() const;
    virtual const AtomicString& formControlType() const;
    virtual const AtomicString& formControlName() const;
    virtual const String& suggestedValue() const;
    virtual String value() const;
    virtual void setValue(const String&, bool sendChangeEvent = false);
    virtual void setValueForUser(const String&);
    virtual void setValueFromRenderer(const String&);

    virtual bool saveFormControlState(String& value) const;
    virtual void restoreFormControlState(const String&);

    virtual void select();
    virtual void accessKeyAction(bool sendToAnyElement);
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void copyNonAttributeProperties(const Element* source);

    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
    virtual void detach();
    virtual bool appendFormData(FormDataList&, bool);
    virtual void reset();

    virtual void defaultEventHandler(Event*);
    virtual void cacheSelection(int start, int end);

    virtual String sanitizeValue(const String& proposedValue) const { return constrainValue(proposedValue); }

    virtual void documentDidBecomeActive();

    virtual void willMoveToNewOwnerDocument();
    virtual void didMoveToNewOwnerDocument();

    bool isConformedToInputMask(const String&);
    bool isConformedToInputMask(UChar, unsigned, bool isUserInput = true);

private:
    friend class WMLCardElement;
    void initialize();

    String validateInputMask(const String&);
    unsigned cursorPositionToMaskIndex(unsigned);
    String constrainValue(const String&) const;

    InputElementData m_data;
    bool m_isPasswordField;
    bool m_isEmptyOk;
    String m_formatMask;
    unsigned m_numOfCharsAllowedByMask;
};

}

#endif
#endif
