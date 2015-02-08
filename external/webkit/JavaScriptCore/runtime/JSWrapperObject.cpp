

#include "config.h"
#include "JSWrapperObject.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(JSWrapperObject);

void JSWrapperObject::markChildren(MarkStack& markStack) 
{
    JSObject::markChildren(markStack);
    if (m_internalValue)
        markStack.append(m_internalValue);
}

} // namespace JSC
