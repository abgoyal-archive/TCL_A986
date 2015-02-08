

#include "config.h"
#include "SearchPopupMenu.h"

#include "AtomicString.h"
#include "NotImplemented.h"

namespace WebCore {

SearchPopupMenu::SearchPopupMenu(PopupMenuClient* client)
    : PopupMenu(client)
{
}

bool SearchPopupMenu::enabled()
{
    return false;
}

void SearchPopupMenu::saveRecentSearches(const AtomicString& name, const Vector<String>& searchItems)
{
    notImplemented();
}

void SearchPopupMenu::loadRecentSearches(const AtomicString& name, Vector<String>& searchItems)
{
    notImplemented();
}

} // namespace WebCore
