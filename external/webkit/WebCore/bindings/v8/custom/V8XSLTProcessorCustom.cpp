

#include "config.h"
#include "V8XSLTProcessor.h"

#include "Document.h"
#include "DocumentFragment.h"
#include "Node.h"

#include "V8Binding.h"
#include "V8Document.h"
#include "V8DocumentFragment.h"
#include "V8Node.h"
#include "V8Proxy.h"
#include "XSLTProcessor.h"

#include <wtf/RefPtr.h>

namespace WebCore {

v8::Handle<v8::Value> V8XSLTProcessor::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XSLTProcessor.Constructor");
    return V8Proxy::constructDOMObject<V8ClassIndex::XSLTPROCESSOR, XSLTProcessor>(args);
}


v8::Handle<v8::Value> V8XSLTProcessor::importStylesheetCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XSLTProcessor.importStylesheet");
    if (!V8Node::HasInstance(args[0]))
        return v8::Undefined();

    XSLTProcessor* imp = V8XSLTProcessor::toNative(args.Holder());

    Node* node = V8Node::toNative(v8::Handle<v8::Object>::Cast(args[0]));
    imp->importStylesheet(node);
    return v8::Undefined();
}


v8::Handle<v8::Value> V8XSLTProcessor::transformToFragmentCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XSLTProcessor.transformToFragment");
    if (!V8Node::HasInstance(args[0]) || !V8Document::HasInstance(args[1]))
        return v8::Undefined();

    XSLTProcessor* imp = V8XSLTProcessor::toNative(args.Holder());

    Node* source = V8Node::toNative(v8::Handle<v8::Object>::Cast(args[0]));
    Document* owner = V8Document::toNative(v8::Handle<v8::Object>::Cast(args[1]));
    RefPtr<DocumentFragment> result = imp->transformToFragment(source, owner);
    return toV8(result.release());
}


v8::Handle<v8::Value> V8XSLTProcessor::transformToDocumentCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XSLTProcessor.transformToDocument");

    if (!V8Node::HasInstance(args[0]))
        return v8::Undefined();

    XSLTProcessor* imp = V8XSLTProcessor::toNative(args.Holder());

    Node* source = V8Node::toNative(v8::Handle<v8::Object>::Cast(args[0]));
    if (!source)
        return v8::Undefined();

    RefPtr<Document> result = imp->transformToDocument(source);
    if (!result)
        return v8::Undefined();

    return toV8(result.release());
}


v8::Handle<v8::Value> V8XSLTProcessor::setParameterCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XSLTProcessor.setParameter");
    if (isUndefinedOrNull(args[1]) || isUndefinedOrNull(args[2]))
        return v8::Undefined();

    XSLTProcessor* imp = V8XSLTProcessor::toNative(args.Holder());

    String namespaceURI = toWebCoreString(args[0]);
    String localName = toWebCoreString(args[1]);
    String value = toWebCoreString(args[2]);
    imp->setParameter(namespaceURI, localName, value);

    return v8::Undefined();
}


v8::Handle<v8::Value> V8XSLTProcessor::getParameterCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XSLTProcessor.getParameter");
    if (isUndefinedOrNull(args[1]))
        return v8::Undefined();

    XSLTProcessor* imp = V8XSLTProcessor::toNative(args.Holder());

    String namespaceURI = toWebCoreString(args[0]);
    String localName = toWebCoreString(args[1]);
    String result = imp->getParameter(namespaceURI, localName);
    if (result.isNull())
        return v8::Undefined();

    return v8String(result);
}

v8::Handle<v8::Value> V8XSLTProcessor::removeParameterCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XSLTProcessor.removeParameter");
    if (isUndefinedOrNull(args[1]))
        return v8::Undefined();

    XSLTProcessor* imp = V8XSLTProcessor::toNative(args.Holder());

    String namespaceURI = toWebCoreString(args[0]);
    String localName = toWebCoreString(args[1]);
    imp->removeParameter(namespaceURI, localName);
    return v8::Undefined();
}

} // namespace WebCore
