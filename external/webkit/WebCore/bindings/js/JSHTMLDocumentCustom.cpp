

#include "config.h"
#include "JSHTMLDocument.h"

#include "CharacterNames.h"
#include "Frame.h"
#include "HTMLAllCollection.h"
#include "HTMLBodyElement.h"
#include "HTMLCollection.h"
#include "HTMLDocument.h"
#include "HTMLElement.h"
#include "HTMLIFrameElement.h"
#include "HTMLNames.h"
#include "JSDOMWindow.h"
#include "JSDOMWindowCustom.h"
#include "JSDOMWindowShell.h"
#include "JSHTMLCollection.h"
#include "SegmentedString.h"
#include "Tokenizer.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

using namespace HTMLNames;

bool JSHTMLDocument::canGetItemsForName(ExecState*, HTMLDocument* document, const Identifier& propertyName)
{
    AtomicStringImpl* atomicPropertyName = AtomicString::find(propertyName);
    return atomicPropertyName && (document->hasNamedItem(atomicPropertyName) || document->hasExtraNamedItem(atomicPropertyName));
}

JSValue JSHTMLDocument::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSHTMLDocument* thisObj = static_cast<JSHTMLDocument*>(asObject(slot.slotBase()));
    HTMLDocument* document = static_cast<HTMLDocument*>(thisObj->impl());

    String name = propertyName;
    RefPtr<HTMLCollection> collection = document->documentNamedItems(name);

    unsigned length = collection->length();
    if (!length)
        return jsUndefined();

    if (length == 1) {
        Node* node = collection->firstItem();

        Frame* frame;
        if (node->hasTagName(iframeTag) && (frame = static_cast<HTMLIFrameElement*>(node)->contentFrame()))
            return toJS(exec, frame);

        return toJS(exec, node);
    } 

    return toJS(exec, collection.get());
}

// Custom attributes

JSValue JSHTMLDocument::all(ExecState* exec) const
{
    // If "all" has been overwritten, return the overwritten value
    JSValue v = getDirect(Identifier(exec, "all"));
    if (v)
        return v;

    return toJS(exec, static_cast<HTMLDocument*>(impl())->all().get());
}

void JSHTMLDocument::setAll(ExecState* exec, JSValue value)
{
    // Add "all" to the property map.
    putDirect(Identifier(exec, "all"), value);
}

// Custom functions

JSValue JSHTMLDocument::open(ExecState* exec, const ArgList& args)
{
    // For compatibility with other browsers, pass open calls with more than 2 parameters to the window.
    if (args.size() > 2) {
        Frame* frame = static_cast<HTMLDocument*>(impl())->frame();
        if (frame) {
            JSDOMWindowShell* wrapper = toJSDOMWindowShell(frame, currentWorld(exec));
            if (wrapper) {
                JSValue function = wrapper->get(exec, Identifier(exec, "open"));
                CallData callData;
                CallType callType = function.getCallData(callData);
                if (callType == CallTypeNone)
                    return throwError(exec, TypeError);
                return JSC::call(exec, function, callType, callData, wrapper, args);
            }
        }
        return jsUndefined();
    }

    // document.open clobbers the security context of the document and
    // aliases it with the active security context.
    Document* activeDocument = asJSDOMWindow(exec->lexicalGlobalObject())->impl()->document();

    // In the case of two parameters or fewer, do a normal document open.
    static_cast<HTMLDocument*>(impl())->open(activeDocument);
    return this;
}

enum NewlineRequirement { DoNotAddNewline, DoAddNewline };

static inline void documentWrite(ExecState* exec, const ArgList& args, HTMLDocument* document, NewlineRequirement addNewline)
{
    // DOM only specifies single string argument, but browsers allow multiple or no arguments.

    size_t size = args.size();

    UString firstString = args.at(0).toString(exec);
    SegmentedString segmentedString = String(firstString);
    if (size != 1) {
        if (!size)
            segmentedString.clear();
        else {
            for (size_t i = 1; i < size; ++i) {
                UString subsequentString = args.at(i).toString(exec);
                segmentedString.append(SegmentedString(String(subsequentString)));
            }
        }
    }
    if (addNewline)
        segmentedString.append(SegmentedString(&newlineCharacter, 1));

    Document* activeDocument = asJSDOMWindow(exec->lexicalGlobalObject())->impl()->document();
    document->write(segmentedString, activeDocument);
}

JSValue JSHTMLDocument::write(ExecState* exec, const ArgList& args)
{
    documentWrite(exec, args, static_cast<HTMLDocument*>(impl()), DoNotAddNewline);
    return jsUndefined();
}

JSValue JSHTMLDocument::writeln(ExecState* exec, const ArgList& args)
{
    documentWrite(exec, args, static_cast<HTMLDocument*>(impl()), DoAddNewline);
    return jsUndefined();
}

} // namespace WebCore
