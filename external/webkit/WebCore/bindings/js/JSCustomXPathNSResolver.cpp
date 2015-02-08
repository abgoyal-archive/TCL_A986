

#include "config.h"
#include "JSCustomXPathNSResolver.h"

#if ENABLE(XPATH)

#include "Document.h"
#include "ExceptionCode.h"
#include "Frame.h"
#include "JSDOMWindowCustom.h"
#include "ScriptController.h"
#include <runtime/JSLock.h>

namespace WebCore {

using namespace JSC;

PassRefPtr<JSCustomXPathNSResolver> JSCustomXPathNSResolver::create(JSC::ExecState* exec, JSC::JSValue value)
{
    if (value.isUndefinedOrNull())
        return 0;

    JSObject* resolverObject = value.getObject();
    if (!resolverObject) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return 0;
    }

    return adoptRef(new JSCustomXPathNSResolver(resolverObject, asJSDOMWindow(exec->dynamicGlobalObject())));
}

JSCustomXPathNSResolver::JSCustomXPathNSResolver(JSObject* customResolver, JSDOMWindow* globalObject)
    : m_customResolver(customResolver)
    , m_globalObject(globalObject)
{
}

JSCustomXPathNSResolver::~JSCustomXPathNSResolver()
{
}

String JSCustomXPathNSResolver::lookupNamespaceURI(const String& prefix)
{
    ASSERT(m_customResolver);

    JSLock lock(SilenceAssertionsOnly);

    ExecState* exec = m_globalObject->globalExec();
        
    JSValue function = m_customResolver->get(exec, Identifier(exec, "lookupNamespaceURI"));
    CallData callData;
    CallType callType = function.getCallData(callData);
    if (callType == CallTypeNone) {
        callType = m_customResolver->getCallData(callData);
        if (callType == CallTypeNone) {
            // FIXME: Pass actual line number and source URL.
            m_globalObject->impl()->console()->addMessage(JSMessageSource, LogMessageType, ErrorMessageLevel, "XPathNSResolver does not have a lookupNamespaceURI method.", 0, String());
            return String();
        }
        function = m_customResolver;
    }

    RefPtr<JSCustomXPathNSResolver> selfProtector(this);

    MarkedArgumentBuffer args;
    args.append(jsString(exec, prefix));

    m_globalObject->globalData()->timeoutChecker.start();
    JSValue retval = JSC::call(exec, function, callType, callData, m_customResolver, args);
    m_globalObject->globalData()->timeoutChecker.stop();

    String result;
    if (exec->hadException())
        reportCurrentException(exec);
    else {
        if (!retval.isUndefinedOrNull())
            result = retval.toString(exec);
    }

    Document::updateStyleForAllDocuments();

    return result;
}

} // namespace WebCore

#endif // ENABLE(XPATH)
