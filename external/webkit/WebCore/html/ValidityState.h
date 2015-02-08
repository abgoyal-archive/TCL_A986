

#ifndef ValidityState_h
#define ValidityState_h

#include "HTMLFormControlElement.h"
#include <wtf/PassOwnPtr.h>

namespace WebCore {

class ValidityState : public Noncopyable {
public:
    static PassOwnPtr<ValidityState> create(HTMLFormControlElement* control)
    {
        return new ValidityState(control);
    }

    void ref() { m_control->ref(); }
    void deref() { m_control->deref(); }

    String validationMessage() const;

    void setCustomErrorMessage(const String& message) { m_customErrorMessage = message; }

    bool valueMissing() const { return m_control->valueMissing(); }
    bool typeMismatch() const;
    bool patternMismatch() const { return m_control->patternMismatch(); }
    bool tooLong() const { return m_control->tooLong(); }
    bool rangeUnderflow() const;
    bool rangeOverflow() const;
    bool stepMismatch() const;
    bool customError() const { return !m_customErrorMessage.isEmpty(); }
    bool valid() const;

private:
    ValidityState(HTMLFormControlElement* control) : m_control(control) { }

    static bool isValidColorString(const String&);
    static bool isValidEmailAddress(const String&);

    HTMLFormControlElement* m_control;
    String m_customErrorMessage;
};

} // namespace WebCore

#endif // ValidityState_h
