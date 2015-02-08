

#include "config.h"
#include "AutocompletePopupMenuClient.h"

#include "HTMLInputElement.h"
#include "WebString.h"
#include "WebVector.h"

using namespace WebCore;

namespace WebKit {

unsigned AutocompletePopupMenuClient::getSuggestionsCount() const
{
    return m_suggestions.size();
}

WebString AutocompletePopupMenuClient::getSuggestion(unsigned listIndex) const
{
    ASSERT(listIndex >= 0 && listIndex < m_suggestions.size());
    return m_suggestions[listIndex];
}

void AutocompletePopupMenuClient::removeSuggestionAtIndex(unsigned listIndex)
{
    ASSERT(listIndex >= 0 && listIndex < m_suggestions.size());
    m_suggestions.remove(listIndex);
}

void AutocompletePopupMenuClient::initialize(
    HTMLInputElement* textField,
    const WebVector<WebString>& suggestions,
    int defaultSuggestionIndex)
{
    ASSERT(defaultSuggestionIndex < static_cast<int>(suggestions.size()));

    // The suggestions must be set before initializing the
    // SuggestionsPopupMenuClient.
    setSuggestions(suggestions);

    SuggestionsPopupMenuClient::initialize(textField, defaultSuggestionIndex);
}

void AutocompletePopupMenuClient::setSuggestions(const WebVector<WebString>& suggestions)
{
    m_suggestions.clear();
    for (size_t i = 0; i < suggestions.size(); ++i)
        m_suggestions.append(suggestions[i]);

    // Try to preserve selection if possible.
    if (getSelectedIndex() >= static_cast<int>(suggestions.size()))
        setSelectedIndex(-1);
}

} // namespace WebKit
