

#ifndef AutoFillPopupMenuClient_h
#define AutoFillPopupMenuClient_h

#include "SuggestionsPopupMenuClient.h"

namespace WebCore {
class HTMLInputElement;
}

namespace WebKit {
class WebString;
template <typename T> class WebVector;

// The AutoFill suggestions popup menu client, used to display name suggestions
// with right-justified labels.
class AutoFillPopupMenuClient : public SuggestionsPopupMenuClient {
public:
    // SuggestionsPopupMenuClient implementation:
    virtual unsigned getSuggestionsCount() const;
    virtual WebString getSuggestion(unsigned listIndex) const;
    virtual void removeSuggestionAtIndex(unsigned listIndex);

    void initialize(WebCore::HTMLInputElement*,
                    const WebVector<WebString>& names,
                    const WebVector<WebString>& labels,
                    int defaultSuggestionIndex);

    void setSuggestions(const WebVector<WebString>& names,
                        const WebVector<WebString>& labels);

private:
    Vector<WebCore::String> m_names;
    Vector<WebCore::String> m_labels;
};

} // namespace WebKit

#endif
