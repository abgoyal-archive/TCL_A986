

#include "config.h"
#include "SearchPopupMenu.h"

#include "AtomicString.h"

namespace WebCore {

SearchPopupMenu::SearchPopupMenu(PopupMenuClient* client)
    : PopupMenu(client)
{
}

void SearchPopupMenu::saveRecentSearches(const AtomicString&, const Vector<String>&)
{
}

void SearchPopupMenu::loadRecentSearches(const AtomicString&, Vector<String>&)
{
}

bool SearchPopupMenu::enabled()
{
    return true;
}

}
