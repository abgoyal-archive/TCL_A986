

#include "config.h"
#include "SearchPopupMenu.h"

#include "NotImplemented.h"

namespace WebCore {

SearchPopupMenu::SearchPopupMenu(PopupMenuClient* client)
    : PopupMenu(client)
{
    notImplemented();
}

void SearchPopupMenu::saveRecentSearches(const AtomicString&, const Vector<String>&)
{
    notImplemented();
}

void SearchPopupMenu::loadRecentSearches(const AtomicString&, Vector<String>&)
{
    notImplemented();
}

bool SearchPopupMenu::enabled()
{
    notImplemented();
    return false;
}

}
