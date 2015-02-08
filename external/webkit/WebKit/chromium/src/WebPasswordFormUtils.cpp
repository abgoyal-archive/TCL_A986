

// Helper to WebPasswordFormData to do the locating of username/password
// fields.
// This method based on Firefox2 code in
//   toolkit/components/passwordmgr/base/nsPasswordManager.cpp

#include "config.h"
#include "WebPasswordFormUtils.h"

#include "HTMLFormElement.h"
#include "HTMLInputElement.h"
#include "HTMLNames.h"
#include "KURL.h"

#include "DOMUtilitiesPrivate.h"

using namespace WebCore;

namespace WebKit {

// Maximum number of password fields we will observe before throwing our
// hands in the air and giving up with a given form.
static const size_t maxPasswords = 3;

void findPasswordFormFields(HTMLFormElement* form, PasswordFormFields* fields)
{
    ASSERT(form);
    ASSERT(fields);

    int firstPasswordIndex = 0;
    // First, find the password fields and activated submit button
    const Vector<HTMLFormControlElement*>& formElements = form->formElements;
    for (size_t i = 0; i < formElements.size(); i++) {
        HTMLFormControlElement* formElement = formElements[i];
        if (formElement->isActivatedSubmit())
            fields->submit = formElement;

        if (!formElement->hasLocalName(HTMLNames::inputTag))
            continue;

        HTMLInputElement* inputElement = toHTMLInputElement(formElement);
        if (!inputElement->isEnabledFormControl())
            continue;

        if ((fields->passwords.size() < maxPasswords)
            && (inputElement->inputType() == HTMLInputElement::PASSWORD)
            && (inputElement->autoComplete())) {
            if (fields->passwords.isEmpty())
                firstPasswordIndex = i;
            fields->passwords.append(inputElement);
        }
    }

    if (!fields->passwords.isEmpty()) {
        // Then, search backwards for the username field
        for (int i = firstPasswordIndex - 1; i >= 0; i--) {
            HTMLFormControlElement* formElement = formElements[i];
            if (!formElement->hasLocalName(HTMLNames::inputTag))
                continue;

            HTMLInputElement* inputElement = toHTMLInputElement(formElement);
            if (!inputElement->isEnabledFormControl())
                continue;

            if ((inputElement->inputType() == HTMLInputElement::TEXT)
                && (inputElement->autoComplete())) {
                fields->userName = inputElement;
                break;
            }
        }
    }
}

} // namespace WebKit
