

#ifndef WMLPageState_h
#define WMLPageState_h

#if ENABLE(WML)
#include "StringHash.h"
#include <wtf/HashMap.h>

namespace WebCore {

class Page;
class String;
class WMLCardElement;

typedef HashMap<String, String> WMLVariableMap;

class WMLPageState {
public:
    WMLPageState(Page*);
    virtual ~WMLPageState();

#ifndef NDEBUG
    void dump();
#endif

    // Reset the browser context
    void reset();

    // Variable handling
    void storeVariable(const String& name, const String& value) { m_variables.set(name, value); }
    void storeVariables(WMLVariableMap& variables) { m_variables = variables; }
    String getVariable(const String& name) const { return m_variables.get(name); }
    bool hasVariables() const { return !m_variables.isEmpty(); }

    Page* page() const { return m_page; }

    // Deck access control
    bool processAccessControlData(const String& dmain, const String& path);
    void resetAccessControlData();

    bool canAccessDeck() const;

private:
    bool hostIsAllowedToAccess(const String&) const;
    bool pathIsAllowedToAccess(const String&) const;

private:
    Page* m_page;
    WMLVariableMap m_variables;
    String m_accessDomain;
    String m_accessPath;
    bool m_hasAccessControlData;
};

}

#endif
#endif
