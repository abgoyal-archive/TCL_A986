

#include "config.h"

#if ENABLE(XSLT)

#include "JSXSLTProcessor.h"

#include "Document.h"
#include "DocumentFragment.h"
#include "JSDocument.h"
#include "JSDocumentFragment.h"
#include "JSNode.h"
#include "Node.h"
#include "PlatformString.h"
#include "XSLTProcessor.h"
#include "JSDOMBinding.h"

using namespace JSC;

namespace WebCore {

JSValue JSXSLTProcessor::importStylesheet(ExecState*, const ArgList& args)
{
    JSValue nodeVal = args.at(0);
    if (nodeVal.inherits(&JSNode::s_info)) {
        JSNode* node = static_cast<JSNode*>(asObject(nodeVal));
        impl()->importStylesheet(node->impl());
        return jsUndefined();
    }
    // Throw exception?
    return jsUndefined();
}

JSValue JSXSLTProcessor::transformToFragment(ExecState* exec, const ArgList& args)
{
    JSValue nodeVal = args.at(0);
    JSValue docVal = args.at(1);
    if (nodeVal.inherits(&JSNode::s_info) && docVal.inherits(&JSDocument::s_info)) {
        WebCore::Node* node = static_cast<JSNode*>(asObject(nodeVal))->impl();
        Document* doc = static_cast<Document*>(static_cast<JSDocument*>(asObject(docVal))->impl());
        return toJS(exec, impl()->transformToFragment(node, doc).get());
    }
    // Throw exception?
    return jsUndefined();
}

JSValue JSXSLTProcessor::transformToDocument(ExecState* exec, const ArgList& args)
{
    JSValue nodeVal = args.at(0);
    if (nodeVal.inherits(&JSNode::s_info)) {
        JSNode* node = static_cast<JSNode*>(asObject(nodeVal));
        RefPtr<Document> resultDocument = impl()->transformToDocument(node->impl());
        if (resultDocument)
            return toJS(exec, resultDocument.get());
        return jsUndefined();
    }
    // Throw exception?
    return jsUndefined();
}

JSValue JSXSLTProcessor::setParameter(ExecState* exec, const ArgList& args)
{
    if (args.at(1).isUndefinedOrNull() || args.at(2).isUndefinedOrNull())
        return jsUndefined(); // Throw exception?
    String namespaceURI = args.at(0).toString(exec);
    String localName = args.at(1).toString(exec);
    String value = args.at(2).toString(exec);
    impl()->setParameter(namespaceURI, localName, value);
    return jsUndefined();
}

JSValue JSXSLTProcessor::getParameter(ExecState* exec, const ArgList& args)
{
    if (args.at(1).isUndefinedOrNull())
        return jsUndefined();
    String namespaceURI = args.at(0).toString(exec);
    String localName = args.at(1).toString(exec);
    String value = impl()->getParameter(namespaceURI, localName);
    return jsStringOrUndefined(exec, value);
}

JSValue JSXSLTProcessor::removeParameter(ExecState* exec, const ArgList& args)
{
    if (args.at(1).isUndefinedOrNull())
        return jsUndefined();
    String namespaceURI = args.at(0).toString(exec);
    String localName = args.at(1).toString(exec);
    impl()->removeParameter(namespaceURI, localName);
    return jsUndefined();
}

} // namespace WebCore

#endif // ENABLE(XSLT)
