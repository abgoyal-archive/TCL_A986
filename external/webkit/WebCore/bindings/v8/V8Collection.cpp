

#include "config.h"
#include "V8Collection.h"

#include "ExceptionCode.h"
#include "HTMLOptionElement.h"
#include "V8HTMLOptionElement.h"

namespace WebCore {

v8::Handle<v8::Value> toOptionsCollectionSetter(uint32_t index, v8::Handle<v8::Value> value, HTMLSelectElement* base)
{
    if (value->IsNull() || value->IsUndefined()) {
        base->remove(index);
        return value;
    }

    ExceptionCode ec = 0;

    // Check that the value is an HTMLOptionElement.  If not, throw a TYPE_MISMATCH_ERR DOMException.
    if (!V8HTMLOptionElement::HasInstance(value)) {
        V8Proxy::setDOMException(TYPE_MISMATCH_ERR);
        return value;
    }

    HTMLOptionElement* element = V8HTMLOptionElement::toNative(v8::Handle<v8::Object>::Cast(value));
    base->setOption(index, element, ec);

    V8Proxy::setDOMException(ec);
    return value;
}

} // namespace WebCore
