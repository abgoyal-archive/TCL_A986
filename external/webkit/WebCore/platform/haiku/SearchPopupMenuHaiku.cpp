

#include "config.h"
#include "SearchPopupMenu.h"

#include "AtomicString.h"
#include "NotImplemented.h"


namespace WebCore {

SearchPopupMenu::SearchPopupMenu(PopupMenuClient* client)
    : PopupMenu(client)
{
}

void SearchPopupMenu::saveRecentSearches(const AtomicString& name, const Vector<String>& searchItems)
{
    notImplemented();
}

void SearchPopupMenu::loadRecentSearches(const AtomicString& name, Vector<String>& searchItems)
{
    notImplemented();
}

bool SearchPopupMenu::enabled()
{
    notImplemented();
    return false;
}

} // namespace WebCore

