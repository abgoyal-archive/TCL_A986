

#ifndef WebPasswordFormData_h
#define WebPasswordFormData_h

#include "WebFormElement.h"
#include "WebString.h"
#include "WebURL.h"

namespace WebKit {

struct WebPasswordFormData {
    // If the provided form is suitable for password completion, isValid() will
    // return true;
    WebPasswordFormData(const WebFormElement&);

    // If creation failed, return false.
    bool isValid() const { return action.isValid(); }

    // The action target of the form. This is the primary data used by the
    // PasswordManager for form autofill; that is, the action of the saved
    // credentials must match the action of the form on the page to be autofilled.
    // If this is empty / not available, it will result in a "restricted"
    // IE-like autofill policy, where we wait for the user to type in his
    // username before autofilling the password. In these cases, after successful
    // login the action URL will automatically be assigned by the
    // PasswordManager.
    //
    // When parsing an HTML form, this must always be set.
    WebURL action;

    // The "Realm" for the sign-on (scheme, host, port for SCHEME_HTML, and
    // contains the HTTP realm for dialog-based forms).
    // The signon_realm is effectively the primary key used for retrieving
    // data from the database, so it must not be empty.
    WebString signonRealm;

    // The URL (minus query parameters) containing the form. This is the primary
    // data used by the PasswordManager to decide (in longest matching prefix
    // fashion) whether or not a given PasswordForm result from the database is a
    // good fit for a particular form on a page, so it must not be empty.
    WebURL origin;

    // The name of the submit button used. Optional; only used in scoring
    // of PasswordForm results from the database to make matches as tight as
    // possible.
    //
    // When parsing an HTML form, this must always be set.
    WebString submitElement;

    // The name of the username input element. Optional (improves scoring).
    //
    // When parsing an HTML form, this must always be set.
    WebString userNameElement;

    // The username. Optional.
    //
    // When parsing an HTML form, this is typically empty unless the site
    // has implemented some form of autofill.
    WebString userNameValue;

    // The name of the password input element, Optional (improves scoring).
    //
    // When parsing an HTML form, this must always be set.
    WebString passwordElement;

    // The password. Required.
    //
    // When parsing an HTML form, this is typically empty.
    WebString passwordValue;

    // If the form was a change password form, the name of the
    // 'old password' input element. Optional.
    WebString oldPasswordElement;

    // The old password. Optional.
    WebString oldPasswordValue;
};

} // namespace WebKit

#endif
