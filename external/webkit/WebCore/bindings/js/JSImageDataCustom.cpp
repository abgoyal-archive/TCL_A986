

#include "config.h"
#include "JSImageData.h"

#include "ImageData.h"
#include "PlatformString.h"

#include <runtime/JSByteArray.h>
#include <wtf/StdLibExtras.h>

using namespace JSC;

namespace WebCore {

JSValue toJS(ExecState* exec, JSDOMGlobalObject* globalObject, ImageData* imageData)
{
    if (!imageData)
        return jsNull();
    
    DOMObject* wrapper = getCachedDOMObjectWrapper(exec, imageData);
    if (wrapper)
        return wrapper;
    
    wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, ImageData, imageData);
    Identifier dataName(exec, "data");
    DEFINE_STATIC_LOCAL(RefPtr<Structure>, cpaStructure, (JSByteArray::createStructure(jsNull())));
    static const ClassInfo cpaClassInfo = { "CanvasPixelArray", 0, 0, 0 };
    wrapper->putDirect(dataName, new (exec) JSByteArray(exec, cpaStructure, imageData->data()->data(), &cpaClassInfo), DontDelete | ReadOnly);
    exec->heap()->reportExtraMemoryCost(imageData->data()->length());
    
    return wrapper;
}

}
