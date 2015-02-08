

#include "config.h"
#include "V8Clipboard.h"

#include "Clipboard.h"
#include "HTMLImageElement.h"
#include "HTMLNames.h"
#include "IntPoint.h"
#include "Node.h"
#include "Element.h"

#include "V8Binding.h"
#include "V8Node.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8Clipboard::typesAccessorGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.Clipboard.types()");
    Clipboard* clipboard = V8Clipboard::toNative(info.Holder());

    HashSet<String> types = clipboard->types();
    if (types.isEmpty())
        return v8::Null();

    v8::Local<v8::Array> result = v8::Array::New(types.size());
    HashSet<String>::const_iterator end = types.end();
    int index = 0;
    for (HashSet<String>::const_iterator it = types.begin(); it != end; ++it, ++index)
        result->Set(v8::Integer::New(index), v8String(*it));

    return result;
}

v8::Handle<v8::Value> V8Clipboard::clearDataCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Clipboard.clearData()");
    Clipboard* clipboard = V8Clipboard::toNative(args.Holder());

    if (!args.Length()) {
        clipboard->clearAllData();
        return v8::Undefined();
    }

    if (args.Length() != 1)
        return throwError("clearData: Invalid number of arguments", V8Proxy::SyntaxError);

    String type = toWebCoreString(args[0]);
    clipboard->clearData(type);
    return v8::Undefined();
}

v8::Handle<v8::Value> V8Clipboard::getDataCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Clipboard.getData()");
    Clipboard* clipboard = V8Clipboard::toNative(args.Holder());

    if (args.Length() != 1)
        return throwError("getData: Invalid number of arguments", V8Proxy::SyntaxError);

    bool success;
    String result = clipboard->getData(toWebCoreString(args[0]), success);
    if (success)
        return v8String(result);

    return v8::Undefined();
}

v8::Handle<v8::Value> V8Clipboard::setDataCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Clipboard.setData()");
    Clipboard* clipboard = V8Clipboard::toNative(args.Holder());

    if (args.Length() != 2)
        return throwError("setData: Invalid number of arguments", V8Proxy::SyntaxError);

    String type = toWebCoreString(args[0]);
    String data = toWebCoreString(args[1]);
    return v8Boolean(clipboard->setData(type, data));
}

v8::Handle<v8::Value> V8Clipboard::setDragImageCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Clipboard.setDragImage()");
    Clipboard* clipboard = V8Clipboard::toNative(args.Holder());

    if (!clipboard->isForDragging())
        return v8::Undefined();

    if (args.Length() != 3)
        return throwError("setDragImage: Invalid number of arguments", V8Proxy::SyntaxError);

    int x = toInt32(args[1]);
    int y = toInt32(args[2]);

    Node* node = 0;
    if (V8Node::HasInstance(args[0]))
        node = V8Node::toNative(v8::Handle<v8::Object>::Cast(args[0]));

    if (!node || !node->isElementNode())
        return throwError("setDragImageFromElement: Invalid first argument");

    if (static_cast<Element*>(node)->hasLocalName(HTMLNames::imgTag) && !node->inDocument())
        clipboard->setDragImage(static_cast<HTMLImageElement*>(node)->cachedImage(), IntPoint(x, y));
    else
        clipboard->setDragImageElement(node, IntPoint(x, y));

    return v8::Undefined();
}

} // namespace WebCore
