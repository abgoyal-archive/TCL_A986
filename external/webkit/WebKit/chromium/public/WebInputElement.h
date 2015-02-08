

#ifndef WebInputElement_h
#define WebInputElement_h

#include "WebElement.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class HTMLInputElement; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {

    // Provides readonly access to some properties of a DOM input element node.
    class WebInputElement : public WebElement {
    public:
        WebInputElement() : WebElement() { }
        WebInputElement(const WebInputElement& n) : WebElement(n) { }

        WebInputElement& operator=(const WebInputElement& n) { WebElement::assign(n); return *this; }
        WEBKIT_API void assign(const WebInputElement& n) { WebElement::assign(n); }

#if WEBKIT_IMPLEMENTATION
        WebInputElement(const WTF::PassRefPtr<WebCore::HTMLInputElement>&);
        WebInputElement& operator=(const WTF::PassRefPtr<WebCore::HTMLInputElement>&);
        operator WTF::PassRefPtr<WebCore::HTMLInputElement>() const;
#endif

        enum InputType {
            Text = 0,
            Password,
            IsIndex,
            CheckBox,
            Radio,
            Submit,
            Reset,
            File,
            Hidden,
            Image,
            Button,
            Search,
            Range,
            Email,
            Number,
            Telephone,
            URL,
            Color,
            Date,
            DateTime,
            DateTimeLocal,
            Month,
            Time,
            Week
        };

        WEBKIT_API bool autoComplete() const;
        WEBKIT_API bool isEnabledFormControl() const;
        WEBKIT_API InputType inputType() const;
        WEBKIT_API WebString formControlType() const;
        WEBKIT_API bool isActivatedSubmit() const;
        WEBKIT_API void setActivatedSubmit(bool);
        WEBKIT_API void setValue(const WebString& value);
        WEBKIT_API WebString value() const;
        WEBKIT_API void setAutofilled(bool);
        WEBKIT_API void dispatchFormControlChangeEvent();
        WEBKIT_API void setSelectionRange(int, int);
        WEBKIT_API WebString name() const;
        // Returns the name that should be used for the specified |element| when
        // storing autofill data.  This is either the field name or its id, an empty
        // string if it has no name and no id.
        WEBKIT_API WebString nameForAutofill() const;
    };

} // namespace WebKit

#endif
