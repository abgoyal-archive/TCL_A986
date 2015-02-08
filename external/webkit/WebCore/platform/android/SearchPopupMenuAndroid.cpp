

#include "config.h"
#include "SearchPopupMenu.h"

namespace WebCore {

// Save the past searches stored in 'searchItems' to a database associated with 'name'
void SearchPopupMenu::saveRecentSearches(const AtomicString& name, const Vector<String>& searchItems) 
{ 
}

// Load past searches associated with 'name' from the database to 'searchItems'
void SearchPopupMenu::loadRecentSearches(const AtomicString& name, Vector<String>& searchItems) 
{ 
}

// Create a search popup menu - not sure what else we have to do here
SearchPopupMenu::SearchPopupMenu(PopupMenuClient* client)
    : PopupMenu(client) 
{ 
}
        
bool SearchPopupMenu::enabled()
{
    return false;
}

}  // namespace WebCore
