

#ifndef InjectedScriptHost_h
#define InjectedScriptHost_h

#include "Console.h"
#include "InspectorController.h"
#include "PlatformString.h"
#include "ScriptState.h"

#include <wtf/HashMap.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class Database;
class InjectedScript;
class InspectorDOMAgent;
class InspectorFrontend;
class JavaScriptCallFrame;
class Node;
class SerializedScriptValue;
class Storage;

class InjectedScriptHost : public RefCounted<InjectedScriptHost>
{
public:
    static PassRefPtr<InjectedScriptHost> create(InspectorController* inspectorController)
    {
        return adoptRef(new InjectedScriptHost(inspectorController));
    }

    ~InjectedScriptHost();

    void setInjectedScriptSource(const String& source) { m_injectedScriptSource = source; }

    InspectorController* inspectorController() { return m_inspectorController; }
    void disconnectController() { m_inspectorController = 0; }

    void clearConsoleMessages();

    void copyText(const String& text);
    Node* nodeForId(long nodeId);
    long pushNodePathToFrontend(Node* node, bool withChildren, bool selectInUI);

    void addNodesToSearchResult(const String& nodeIds);
    long pushNodeByPathToFrontend(const String& path);

#if ENABLE(JAVASCRIPT_DEBUGGER) && USE(JSC)
    JavaScriptCallFrame* currentCallFrame() const;
#endif
#if ENABLE(DATABASE)
    Database* databaseForId(long databaseId);
    void selectDatabase(Database* database);
#endif
#if ENABLE(DOM_STORAGE)
    void selectDOMStorage(Storage* storage);
#endif
    void reportDidDispatchOnInjectedScript(long callId, SerializedScriptValue* result, bool isException);

    InjectedScript injectedScriptFor(ScriptState*);
    InjectedScript injectedScriptForId(long);
    void discardInjectedScripts();
    void releaseWrapperObjectGroup(long injectedScriptId, const String& objectGroup);

private:
    InjectedScriptHost(InspectorController* inspectorController);
    InspectorDOMAgent* inspectorDOMAgent();
    InspectorFrontend* inspectorFrontend();

    InspectorController* m_inspectorController;
    String m_injectedScriptSource;
    long m_nextInjectedScriptId;
    typedef HashMap<long, InjectedScript> IdToInjectedScriptMap;
    IdToInjectedScriptMap m_idToInjectedScript;
};

} // namespace WebCore

#endif // !defined(InjectedScriptHost_h)
