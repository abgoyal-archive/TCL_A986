

#ifndef WebSearchableFormData_h
#define WebSearchableFormData_h

#include "WebString.h"
#include "WebURL.h"

namespace WebKit {
class WebFormElement;

// SearchableFormData encapsulates a URL and encoding of an INPUT field that
// corresponds to a searchable form request.
class WebSearchableFormData {
public:
    // If the provided form is suitable for automated searching, isValid()
    // will return false.
    WebSearchableFormData(const WebFormElement&);

    bool isValid() { return m_url.isValid(); }

    // URL for the searchable form request.
    const WebURL& url() const
    {
        return m_url;
    }

    // Encoding used to encode the form parameters; never empty.
    const WebString& encoding() const
    {
        return m_encoding;
    }

private:
    WebURL m_url;
    WebString m_encoding;
};

} // namespace WebKit

#endif
