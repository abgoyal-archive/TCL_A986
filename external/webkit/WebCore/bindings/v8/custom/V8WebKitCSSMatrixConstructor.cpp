

#include "config.h"
#include "V8WebKitCSSMatrix.h"

#include "Document.h"
#include "DocumentFragment.h"
#include "Node.h"

#include "V8Binding.h"
#include "V8Document.h"
#include "V8Node.h"
#include "V8Proxy.h"

#include <wtf/RefPtr.h>

namespace WebCore {

v8::Handle<v8::Value> V8WebKitCSSMatrix::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebKitCSSMatrix.Constructor");

    if (!args.IsConstructCall())
        return throwError("DOM object constructor cannot be called as a function.");

    // FIXME: The logic here is almost exact duplicate of V8::constructDOMObject.
    // Consider refactoring to reduce duplication.
    String cssValue;
    if (args.Length() >= 1)
        cssValue = toWebCoreString(args[0]);

    ExceptionCode ec = 0;
    RefPtr<WebKitCSSMatrix> matrix = WebKitCSSMatrix::create(cssValue, ec);
    if (ec)
        throwError(ec);

    // Transform the holder into a wrapper object for the matrix.
    V8DOMWrapper::setDOMWrapper(args.Holder(), V8ClassIndex::ToInt(V8ClassIndex::WEBKITCSSMATRIX), matrix.get());
    return toV8(matrix.release(), args.Holder());
}

} // namespace WebCore
