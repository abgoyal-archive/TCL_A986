

#ifndef InspectorFrontend_h
#define InspectorFrontend_h

#include "ScriptArray.h"
#include "ScriptObject.h"
#include "ScriptState.h"
#include <wtf/PassOwnPtr.h>

#if ENABLE(JAVASCRIPT_DEBUGGER) && USE(JSC)
namespace JSC {
    class JSValue;
    class SourceCode;
    class UString;
}
#endif

namespace WebCore {
    class ConsoleMessage;
    class Database;
    class Frame;
    class InspectorController;
    class InspectorResource;
    class Node;
    class ScriptString;
    class SerializedScriptValue;
    class Storage;

    class InspectorFrontend : public Noncopyable {
    public:
        InspectorFrontend(InspectorController* inspectorController, ScriptObject webInspector);
        ~InspectorFrontend();

        ScriptArray newScriptArray();
        ScriptObject newScriptObject();

        void didCommitLoad();

        void populateFrontendSettings(const String& settings);

        void updateConsoleMessageExpiredCount(unsigned count);
        void addConsoleMessage(const ScriptObject& messageObj, const Vector<ScriptString>& frames, ScriptState*, const Vector<ScriptValue> arguments, const String& message);
        void updateConsoleMessageRepeatCount(unsigned count);
        void clearConsoleMessages();

        bool updateResource(unsigned long identifier, const ScriptObject& resourceObj);
        void removeResource(unsigned long identifier);
        void didGetResourceContent(int callId, const String& content);

        void updateFocusedNode(long nodeId);
        void setAttachedWindow(bool attached);
        void showPanel(int panel);
        void populateInterface();
        void reset();

        void resourceTrackingWasEnabled();
        void resourceTrackingWasDisabled();

#if ENABLE(JAVASCRIPT_DEBUGGER) && USE(JSC)
        void attachDebuggerWhenShown();
        void debuggerWasEnabled();
        void debuggerWasDisabled();
        void parsedScriptSource(const JSC::SourceCode&);
        void failedToParseScriptSource(const JSC::SourceCode&, int errorLine, const JSC::UString& errorMessage);
        void pausedScript(SerializedScriptValue* callFrames);
        void resumedScript();
#endif
#if ENABLE(JAVASCRIPT_DEBUGGER)
        void profilerWasEnabled();
        void profilerWasDisabled();
        void addProfileHeader(const ScriptValue& profile);
        void setRecordingProfile(bool isProfiling);
        void didGetProfileHeaders(int callId, const ScriptArray& headers);
        void didGetProfile(int callId, const ScriptValue& profile);
#endif

#if ENABLE(DATABASE)
        bool addDatabase(const ScriptObject& dbObj);
        void selectDatabase(int databaseId);
        void didGetDatabaseTableNames(int callId, const ScriptArray& tableNames);
#endif
        
#if ENABLE(DOM_STORAGE)
        bool addDOMStorage(const ScriptObject& domStorageObj);
        void selectDOMStorage(int storageId);
        void didGetDOMStorageEntries(int callId, const ScriptArray& entries);
        void didSetDOMStorageItem(int callId, bool success);
        void didRemoveDOMStorageItem(int callId, bool success);
        void updateDOMStorage(int storageId);
#endif

        void setDocument(const ScriptObject& root);
        void setDetachedRoot(const ScriptObject& root);
        void setChildNodes(int parentId, const ScriptArray& nodes);
        void childNodeCountUpdated(int id, int newValue);
        void childNodeInserted(int parentId, int prevId, const ScriptObject& node);
        void childNodeRemoved(int parentId, int id);
        void attributesUpdated(int id, const ScriptArray& attributes);
        void didGetChildNodes(int callId);
        void didApplyDomChange(int callId, bool success);
        void didGetEventListenersForNode(int callId, int nodeId, ScriptArray& listenersArray);
        void didRemoveNode(int callId, int nodeId);

        void timelineProfilerWasStarted();
        void timelineProfilerWasStopped();
        void addRecordToTimeline(const ScriptObject&);

        void didGetCookies(int callId, const ScriptArray& cookies, const String& cookiesString);
        void didDispatchOnInjectedScript(int callId, SerializedScriptValue* result, bool isException);

        void addNodesToSearchResult(const String& nodeIds);

        void contextMenuItemSelected(int itemId);
        void contextMenuCleared();

        ScriptState* scriptState() const { return m_webInspector.scriptState(); }

        void evaluateForTestInFrontend(int callId, const String& script);
    private:
        void callSimpleFunction(const String& functionName);
        InspectorController* m_inspectorController;
        ScriptObject m_webInspector;
    };

} // namespace WebCore

#endif // !defined(InspectorFrontend_h)
