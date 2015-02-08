

#include "config.h"
#include "AutoFillPopupMenuClient.h"

#include "HTMLInputElement.h"
#include "WebString.h"
#include "WebVector.h"

using namespace WebCore;

namespace WebKit {

unsigned AutoFillPopupMenuClient::getSuggestionsCount() const
{
    return m_names.size();
}

WebString AutoFillPopupMenuClient::getSuggestion(unsigned listIndex) const
{
    // FIXME: Modify the PopupMenu to add the label in gray right-justified.
    ASSERT(listIndex >= 0 && listIndex < m_names.size());
    return m_names[listIndex] + String(" (") + m_labels[listIndex] + String(")");
}

void AutoFillPopupMenuClient::removeSuggestionAtIndex(unsigned listIndex)
{
    // FIXME: Do we want to remove AutoFill suggestions?
    ASSERT(listIndex >= 0 && listIndex < m_names.size());
    m_names.remove(listIndex);
    m_labels.remove(listIndex);
}

void AutoFillPopupMenuClient::initialize(
    HTMLInputElement* textField,
    const WebVector<WebString>& names,
    const WebVector<WebString>& labels,
    int defaultSuggestionIndex)
{
    ASSERT(names.size() == labels.size());
    ASSERT(defaultSuggestionIndex < static_cast<int>(names.size()));

    // The suggestions must be set before initializing the
    // SuggestionsPopupMenuClient.
    setSuggestions(names, labels);

    SuggestionsPopupMenuClient::initialize(textField, defaultSuggestionIndex);
}

void AutoFillPopupMenuClient::setSuggestions(const WebVector<WebString>& names,
                                             const WebVector<WebString>& labels)
{
    ASSERT(names.size() == labels.size());

    m_names.clear();
    m_labels.clear();
    for (size_t i = 0; i < names.size(); ++i) {
        m_names.append(names[i]);
        m_labels.append(labels[i]);
    }

    // Try to preserve selection if possible.
    if (getSelectedIndex() >= static_cast<int>(names.size()))
        setSelectedIndex(-1);
}

} // namespace WebKit
