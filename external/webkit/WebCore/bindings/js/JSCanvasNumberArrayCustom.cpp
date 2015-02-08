

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "JSCanvasNumberArray.h"

#include "CanvasNumberArray.h"

using namespace JSC;

namespace WebCore {

JSValue JSCanvasNumberArray::getByIndex(JSC::ExecState* exec, unsigned int index)
{
    JSC::JSValue result = jsNumber(exec, impl()->item(index));
    return result;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
