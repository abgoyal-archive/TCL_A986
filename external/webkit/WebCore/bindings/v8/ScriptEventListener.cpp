

#include "config.h"
#include "ScriptEventListener.h"

#include "Attribute.h"
#include "Document.h"
#include "EventListener.h"
#include "Frame.h"
#include "ScriptScope.h"
#include "Tokenizer.h"
#include "V8AbstractEventListener.h"
#include "V8Binding.h"
#include "XSSAuditor.h"

namespace WebCore {

PassRefPtr<V8LazyEventListener> createAttributeEventListener(Node* node, Attribute* attr)
{
    ASSERT(node);
    ASSERT(attr);
    if (attr->isNull())
        return 0;

    int lineNumber = 1;
    int columnNumber = 0;
    String sourceURL;

    if (Frame* frame = node->document()->frame()) {
        ScriptController* scriptController = frame->script();
        if (!scriptController->canExecuteScripts())
            return 0;

        if (!scriptController->xssAuditor()->canCreateInlineEventListener(attr->localName().string(), attr->value())) {
            // This script is not safe to execute.
            return 0;
        }

        if (frame->document()->tokenizer()) {
            // FIXME: Change to use script->eventHandlerLineNumber() when implemented.
            lineNumber = frame->document()->tokenizer()->lineNumber();
            columnNumber = frame->document()->tokenizer()->columnNumber();
        }
        sourceURL = node->document()->url().string();
    }

    return V8LazyEventListener::create(attr->localName().string(), node->isSVGElement(), attr->value(), sourceURL, lineNumber, columnNumber, WorldContextHandle(UseMainWorld));
}

PassRefPtr<V8LazyEventListener> createAttributeEventListener(Frame* frame, Attribute* attr)
{
    if (!frame)
        return 0;

    ASSERT(attr);
    if (attr->isNull())
        return 0;

    int lineNumber = 1;
    int columnNumber = 0;
    String sourceURL;

    ScriptController* scriptController = frame->script();
    if (!scriptController->canExecuteScripts())
        return 0;

    if (!scriptController->xssAuditor()->canCreateInlineEventListener(attr->localName().string(), attr->value())) {
        // This script is not safe to execute.
        return 0;
    }

    if (frame->document()->tokenizer()) {
        // FIXME: Change to use script->eventHandlerLineNumber() when implemented.
        lineNumber = frame->document()->tokenizer()->lineNumber();
        columnNumber = frame->document()->tokenizer()->columnNumber();
    }
    sourceURL = frame->document()->url().string();
    return V8LazyEventListener::create(attr->localName().string(), frame->document()->isSVGDocument(), attr->value(), sourceURL, lineNumber, columnNumber, WorldContextHandle(UseMainWorld));
}

String getEventListenerHandlerBody(ScriptExecutionContext* context, ScriptState* scriptState, EventListener* listener)
{
    if (listener->type() != EventListener::JSEventListenerType)
        return "";

    ScriptScope scope(scriptState);
    V8AbstractEventListener* v8Listener = static_cast<V8AbstractEventListener*>(listener);
    v8::Handle<v8::Object> function = v8Listener->getListenerObject(context);
    if (function.IsEmpty())
        return "";

    return toWebCoreStringWithNullCheck(function);
}

} // namespace WebCore
