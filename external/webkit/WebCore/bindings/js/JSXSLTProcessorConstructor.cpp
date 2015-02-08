

#include "config.h"

#if ENABLE(XSLT)

#include "JSXSLTProcessorConstructor.h"

#include "JSXSLTProcessor.h"
#include "XSLTProcessor.h"
#include <wtf/RefPtr.h>

using namespace JSC;

namespace WebCore {

ASSERT_CLASS_FITS_IN_CELL(JSXSLTProcessorConstructor);

const ClassInfo JSXSLTProcessorConstructor::s_info = { "XSLTProcessorConsructor", 0, 0, 0 };

JSXSLTProcessorConstructor::JSXSLTProcessorConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSXSLTProcessorConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSXSLTProcessorPrototype::self(exec, globalObject), None);
}

static JSObject* constructXSLTProcessor(ExecState* exec, JSObject* constructor, const ArgList&)
{
    JSXSLTProcessorConstructor* jsConstructor = static_cast<JSXSLTProcessorConstructor*>(constructor);
    return CREATE_DOM_OBJECT_WRAPPER(exec, jsConstructor->globalObject(), XSLTProcessor, XSLTProcessor::create().get());
}

ConstructType JSXSLTProcessorConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructXSLTProcessor;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(XSLT)
