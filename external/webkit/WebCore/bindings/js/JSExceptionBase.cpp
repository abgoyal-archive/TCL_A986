

#include "config.h"
#include "JSExceptionBase.h"

#include "JSDOMCoreException.h"
#include "JSEventException.h"
#include "JSRangeException.h"
#include "JSXMLHttpRequestException.h"
#if ENABLE(SVG)
#include "JSSVGException.h"
#endif
#if ENABLE(XPATH)
#include "JSXPathException.h"
#endif
 
namespace WebCore {
 
ExceptionBase* toExceptionBase(JSC::JSValue value)
{
    if (DOMCoreException* domException = toDOMCoreException(value))
        return reinterpret_cast<ExceptionBase*>(domException);
    if (RangeException* rangeException = toRangeException(value))
        return reinterpret_cast<ExceptionBase*>(rangeException);
    if (EventException* eventException = toEventException(value))
        return reinterpret_cast<ExceptionBase*>(eventException);
    if (XMLHttpRequestException* xmlHttpException = toXMLHttpRequestException(value))
        return reinterpret_cast<ExceptionBase*>(xmlHttpException);
#if ENABLE(SVG)
    if (SVGException* svgException = toSVGException(value))
        return reinterpret_cast<ExceptionBase*>(svgException);
#endif
#if ENABLE(XPATH)
    if (XPathException* pathException = toXPathException(value))
        return reinterpret_cast<ExceptionBase*>(pathException);
#endif
    
    return 0;
}

} // namespace WebCore
