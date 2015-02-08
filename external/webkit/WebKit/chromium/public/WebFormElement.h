

#ifndef WebFormElement_h
#define WebFormElement_h

#include "WebElement.h"
#include "WebInputElement.h"
#include "WebVector.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class HTMLFormElement; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {
    // A container for passing around a reference to a form element.  Provides
    // some information about the form.
    class WebFormElement : public WebElement {
    public:
        ~WebFormElement() { reset(); }

        WebFormElement() : WebElement() { }
        WebFormElement(const WebFormElement& e) : WebElement(e) { }

        WebFormElement& operator=(const WebFormElement& e)
        {
            WebElement::assign(e);
            return *this;
        }
        WEBKIT_API void assign(const WebFormElement& e) { WebElement::assign(e); }

#if WEBKIT_IMPLEMENTATION
        WebFormElement(const WTF::PassRefPtr<WebCore::HTMLFormElement>&);
        WebFormElement& operator=(const WTF::PassRefPtr<WebCore::HTMLFormElement>&);
        operator WTF::PassRefPtr<WebCore::HTMLFormElement>() const;
#endif

        WEBKIT_API bool autoComplete() const;
        WEBKIT_API WebString action() const;
        WEBKIT_API WebString name() const;
        WEBKIT_API WebString method() const;
        WEBKIT_API void submit();
        // FIXME: Deprecate and replace with WebVector<WebElement>.
        WEBKIT_API void getNamedElements(const WebString&, WebVector<WebNode>&);
        WEBKIT_API void getInputElements(WebVector<WebInputElement>&) const;
    };

} // namespace WebKit

#endif
