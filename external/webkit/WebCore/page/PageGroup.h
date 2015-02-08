

#ifndef PageGroup_h
#define PageGroup_h

#include <wtf/HashSet.h>
#include <wtf/Noncopyable.h>
#include "LinkHash.h"
#include "StringHash.h"
#include "UserScript.h"
#include "UserStyleSheet.h"

namespace WebCore {

    class KURL;
    class Page;
    class StorageNamespace;

    class PageGroup : public Noncopyable {
    public:
        PageGroup(const String& name);
        PageGroup(Page*);
        ~PageGroup();

        static PageGroup* pageGroup(const String& groupName);
        static void closeLocalStorage();
        
#if ENABLE(DOM_STORAGE) && defined(ANDROID)
        static void clearDomStorage();
#endif

        const HashSet<Page*>& pages() const { return m_pages; }

        void addPage(Page*);
        void removePage(Page*);

        bool isLinkVisited(LinkHash);

        void addVisitedLink(const KURL&);
        void addVisitedLink(const UChar*, size_t);
        void removeVisitedLinks();

        static void setShouldTrackVisitedLinks(bool);
        static void removeAllVisitedLinks();

        const String& name() { return m_name; }
        unsigned identifier() { return m_identifier; }

#if ENABLE(DOM_STORAGE)
        StorageNamespace* localStorage();
        bool hasLocalStorage() { return m_localStorage; }
#endif

        void addUserScriptToWorld(DOMWrapperWorld*, const String& source, const KURL&, 
                                  PassOwnPtr<Vector<String> > whitelist, PassOwnPtr<Vector<String> > blacklist,
                                  UserScriptInjectionTime);
        void addUserStyleSheetToWorld(DOMWrapperWorld*, const String& source, const KURL&,
                               PassOwnPtr<Vector<String> > whitelist, PassOwnPtr<Vector<String> > blacklist);
        
        void removeUserScriptFromWorld(DOMWrapperWorld*, const KURL&);
        void removeUserStyleSheetFromWorld(DOMWrapperWorld*, const KURL&);
        
        void removeUserScriptsFromWorld(DOMWrapperWorld*);
        void removeUserStyleSheetsFromWorld(DOMWrapperWorld*);
    
        void removeAllUserContent();
        
        const UserScriptMap* userScripts() const { return m_userScripts.get(); }
        const UserStyleSheetMap* userStyleSheets() const { return m_userStyleSheets.get(); }

    private:
        void addVisitedLink(LinkHash stringHash);

#if ENABLE(DOM_STORAGE) && defined(ANDROID)
        void removeLocalStorage();
#endif

        String m_name;

        HashSet<Page*> m_pages;

        HashSet<LinkHash, LinkHashHash> m_visitedLinkHashes;
        bool m_visitedLinksPopulated;

        unsigned m_identifier;
#if ENABLE(DOM_STORAGE)
        RefPtr<StorageNamespace> m_localStorage;
#endif

        OwnPtr<UserScriptMap> m_userScripts;
        OwnPtr<UserStyleSheetMap> m_userStyleSheets;
    };

} // namespace WebCore
    
#endif // PageGroup_h
