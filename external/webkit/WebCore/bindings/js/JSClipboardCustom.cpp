

#include "config.h"
#include "JSClipboard.h"

#include "Clipboard.h"
#include "Element.h"
#include "HTMLImageElement.h"
#include "HTMLNames.h"
#include "IntPoint.h"
#include "JSNode.h"
#include "Node.h"
#include "PlatformString.h"
#include "StringHash.h"
#include <runtime/ArrayPrototype.h>
#include <runtime/Error.h>
#include <wtf/HashSet.h>

using namespace JSC;

namespace WebCore {

using namespace HTMLNames;

JSValue JSClipboard::types(ExecState* exec) const
{
    Clipboard* clipboard = impl();

    HashSet<String> types = clipboard->types();
    if (types.isEmpty())
        return jsNull();

    MarkedArgumentBuffer list;
    HashSet<String>::const_iterator end = types.end();
    for (HashSet<String>::const_iterator it = types.begin(); it != end; ++it)
        list.append(jsString(exec, UString(*it)));
    return constructArray(exec, list);
}

JSValue JSClipboard::clearData(ExecState* exec, const ArgList& args)
{
    Clipboard* clipboard = impl();

    if (args.size() == 0) {
        clipboard->clearAllData();
        return jsUndefined();
    }

    if (args.size() == 1) {
        clipboard->clearData(args.at(0).toString(exec));
        return jsUndefined();
    }

    // FIXME: It does not match the rest of the JS bindings to throw on invalid number of arguments. 
    return throwError(exec, SyntaxError, "clearData: Invalid number of arguments");
}

JSValue JSClipboard::getData(ExecState* exec, const ArgList& args)
{
    // FIXME: It does not match the rest of the JS bindings to throw on invalid number of arguments.
    if (args.size() != 1)
        return throwError(exec, SyntaxError, "getData: Invalid number of arguments");

    Clipboard* clipboard = impl();

    bool success;
    String result = clipboard->getData(args.at(0).toString(exec), success);
    if (!success)
        return jsUndefined();

    return jsString(exec, result);
}

JSValue JSClipboard::setData(ExecState* exec, const ArgList& args)
{
    Clipboard* clipboard = impl();

    // FIXME: It does not match the rest of the JS bindings to throw on invalid number of arguments. 
    if (args.size() != 2)
        return throwError(exec, SyntaxError, "setData: Invalid number of arguments");

    return jsBoolean(clipboard->setData(args.at(0).toString(exec), args.at(1).toString(exec)));
}

JSValue JSClipboard::setDragImage(ExecState* exec, const ArgList& args)
{
    Clipboard* clipboard = impl();

    if (!clipboard->isForDragging())
        return jsUndefined();

    // FIXME: It does not match the rest of the JS bindings to throw on invalid number of arguments. 
    if (args.size() != 3)
        return throwError(exec, SyntaxError, "setDragImage: Invalid number of arguments");

    int x = args.at(1).toInt32(exec);
    int y = args.at(2).toInt32(exec);

    // See if they passed us a node
    Node* node = toNode(args.at(0));
    if (!node)
        return throwError(exec, TypeError);

    // FIXME: This should probably be a TypeError. 
    if (!node->isElementNode())
        return throwError(exec, SyntaxError, "setDragImageFromElement: Invalid first argument");

    if (static_cast<Element*>(node)->hasLocalName(imgTag) && !node->inDocument())
        clipboard->setDragImage(static_cast<HTMLImageElement*>(node)->cachedImage(), IntPoint(x, y));
    else
        clipboard->setDragImageElement(node, IntPoint(x, y));

    return jsUndefined();
}

} // namespace WebCore
