

#include "config.h"
#include "SearchPopupMenu.h"

#include "NotImplemented.h"

namespace WebCore {

SearchPopupMenu::SearchPopupMenu(PopupMenuClient* client)
    : PopupMenu(client)
{
    notImplemented();
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
    return true;
}

}

