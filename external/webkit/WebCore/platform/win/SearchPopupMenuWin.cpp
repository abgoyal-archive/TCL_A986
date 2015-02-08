

#include "config.h"
#include "SearchPopupMenu.h"

#include "AtomicString.h"
#include <wtf/RetainPtr.h>

namespace WebCore {

SearchPopupMenu::SearchPopupMenu(PopupMenuClient* client)
    : PopupMenu(client)
{
}

bool SearchPopupMenu::enabled()
{
    return true;
}

static RetainPtr<CFStringRef> autosaveKey(const String& name)
{
    String key = "com.apple.WebKit.searchField:" + name;
    return RetainPtr<CFStringRef>(AdoptCF, key.createCFString());
}

void SearchPopupMenu::saveRecentSearches(const AtomicString& name, const Vector<String>& searchItems)
{
    if (name.isEmpty())
        return;

    RetainPtr<CFMutableArrayRef> items;

    size_t size = searchItems.size();
    if (size) {
        items.adoptCF(CFArrayCreateMutable(0, size, &kCFTypeArrayCallBacks));
        for (size_t i = 0; i < size; ++i) {
            RetainPtr<CFStringRef> item(AdoptCF, searchItems[i].createCFString());
            CFArrayAppendValue(items.get(), item.get());
        }
    }

    CFPreferencesSetAppValue(autosaveKey(name).get(), items.get(), kCFPreferencesCurrentApplication);
    CFPreferencesAppSynchronize(kCFPreferencesCurrentApplication);
}

void SearchPopupMenu::loadRecentSearches(const AtomicString& name, Vector<String>& searchItems)
{
    if (name.isEmpty())
        return;

    searchItems.clear();
    RetainPtr<CFArrayRef> items(AdoptCF, reinterpret_cast<CFArrayRef>(CFPreferencesCopyAppValue(autosaveKey(name).get(), kCFPreferencesCurrentApplication)));

    if (!items || CFGetTypeID(items.get()) != CFArrayGetTypeID())
        return;

    size_t size = CFArrayGetCount(items.get());
    for (size_t i = 0; i < size; ++i) {
        CFStringRef item = (CFStringRef)CFArrayGetValueAtIndex(items.get(), i);
        if (CFGetTypeID(item) == CFStringGetTypeID())
            searchItems.append(item);
    }
}

}
