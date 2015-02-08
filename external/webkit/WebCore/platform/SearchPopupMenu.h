

#ifndef SearchPopupMenu_h
#define SearchPopupMenu_h

#include "PopupMenu.h"
#include <wtf/Vector.h>

namespace WebCore {

class AtomicString;

class SearchPopupMenu : public PopupMenu {
public:
    static PassRefPtr<SearchPopupMenu> create(PopupMenuClient* client) { return adoptRef(new SearchPopupMenu(client)); }

    void saveRecentSearches(const AtomicString& name, const Vector<String>& searchItems);
    void loadRecentSearches(const AtomicString& name, Vector<String>& searchItems);

    bool enabled();
    
private:
    SearchPopupMenu(PopupMenuClient*);
};

}

#endif
