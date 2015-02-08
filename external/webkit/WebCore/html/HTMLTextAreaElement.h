

#ifndef HTMLTextAreaElement_h
#define HTMLTextAreaElement_h

#include "HTMLFormControlElement.h"

namespace WebCore {

class BeforeTextInsertedEvent;
class VisibleSelection;

class HTMLTextAreaElement : public HTMLTextFormControlElement {
public:
    HTMLTextAreaElement(const QualifiedName&, Document*, HTMLFormElement* = 0);

    virtual bool checkDTD(const Node* newChild) { return newChild->isTextNode(); }

    int cols() const { return m_cols; }
    int rows() const { return m_rows; }

    bool shouldWrapText() const { return m_wrap != NoWrap; }

    virtual bool isEnumeratable() const { return true; }

    virtual const AtomicString& formControlType() const;

    virtual bool saveFormControlState(String& value) const;
    virtual void restoreFormControlState(const String&);

    bool readOnly() const { return isReadOnlyFormControl(); }

    virtual bool isTextFormControl() const { return true; }

    virtual bool valueMissing() const { return isRequiredFormControl() && !disabled() && !readOnly() && value().isEmpty(); }

    virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);
    virtual void parseMappedAttribute(MappedAttribute*);
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
    virtual bool appendFormData(FormDataList&, bool);
    virtual void reset();
    virtual void defaultEventHandler(Event*);
    virtual bool isMouseFocusable() const;
    virtual bool isKeyboardFocusable(KeyboardEvent*) const;
    virtual void updateFocusAppearance(bool restorePreviousSelection);

    String value() const;
    void setValue(const String&);
    String defaultValue() const;
    void setDefaultValue(const String&);
    int textLength() const { return value().length(); }
    int maxLength() const;
    void setMaxLength(int, ExceptionCode&);
    virtual bool tooLong() const;
    
    void rendererWillBeDestroyed();
    
    virtual void accessKeyAction(bool sendToAnyElement);
    
    const AtomicString& accessKey() const;
    void setAccessKey(const String&);

    void setCols(int);
    void setRows(int);
    
    void cacheSelection(int s, int e) { m_cachedSelectionStart = s; m_cachedSelectionEnd = e; };

    virtual bool shouldUseInputMethod() const;

private:
    enum WrapMethod { NoWrap, SoftWrap, HardWrap };

    void handleBeforeTextInsertedEvent(BeforeTextInsertedEvent*) const;
    static String sanitizeUserInputValue(const String&, unsigned maxLength);
    void updateValue() const;
    void setNonDirtyValue(const String&);

    virtual bool supportsPlaceholder() const { return true; }
    virtual bool isEmptyValue() const { return value().isEmpty(); }
    virtual int cachedSelectionStart() const { return m_cachedSelectionStart; }
    virtual int cachedSelectionEnd() const { return m_cachedSelectionEnd; }

    virtual bool isOptionalFormControl() const { return !isRequiredFormControl(); }
    virtual bool isRequiredFormControl() const { return required(); }

    int m_rows;
    int m_cols;
    WrapMethod m_wrap;
    mutable String m_value;
    int m_cachedSelectionStart;
    int m_cachedSelectionEnd;
    mutable bool m_isDirty;
};

} //namespace

#endif
