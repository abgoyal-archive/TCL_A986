

#ifndef WebPasswordFormUtils_h
#define WebPasswordFormUtils_h

#include <wtf/Vector.h>

namespace WebCore {
class HTMLInputElement;
class HTMLFormControlElement;
class HTMLFormElement;
}

namespace WebKit {

// Helper structure to locate username, passwords and submit fields.
struct PasswordFormFields {
    WebCore::HTMLInputElement* userName;
    Vector<WebCore::HTMLInputElement*> passwords;
    WebCore::HTMLFormControlElement* submit;
    PasswordFormFields() : userName(0), submit(0) { }
};

void findPasswordFormFields(WebCore::HTMLFormElement* form,
                            PasswordFormFields* fields);

} // namespace WebKit

#endif
