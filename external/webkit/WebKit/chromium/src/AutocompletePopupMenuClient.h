

#ifndef AutocompletePopupMenuClient_h
#define AutocompletePopupMenuClient_h
 
#include "SuggestionsPopupMenuClient.h"

namespace WebCore {
class HTMLInputElement;
}

namespace WebKit {
class WebString;
template <typename T> class WebVector;

// The Autocomplete suggestions popup menu client, used to display a list of
// autocomplete suggestions.
class AutocompletePopupMenuClient : public SuggestionsPopupMenuClient {
public:
    // SuggestionsPopupMenuClient implementation:
    virtual unsigned getSuggestionsCount() const;
    virtual WebString getSuggestion(unsigned listIndex) const;
    virtual void removeSuggestionAtIndex(unsigned listIndex);

    void initialize(WebCore::HTMLInputElement*,
                    const WebVector<WebString>& suggestions,
                    int defaultSuggestionIndex);

    void setSuggestions(const WebVector<WebString>&);

private:
    Vector<WebCore::String> m_suggestions;
};

} // namespace WebKit

#endif
