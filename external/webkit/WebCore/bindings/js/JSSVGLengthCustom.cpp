

#include "config.h"

#if ENABLE(SVG)
#include "JSSVGLength.h"

using namespace JSC;

namespace WebCore {

JSValue JSSVGLength::value(ExecState* exec) const
{
    JSSVGPODTypeWrapper<SVGLength>* imp = impl();
    SVGElement* context = JSSVGContextCache::svgContextForDOMObject(const_cast<JSSVGLength*>(this));

    SVGLength podImp(*imp);
    return jsNumber(exec, podImp.value(context));
}

JSValue JSSVGLength::convertToSpecifiedUnits(ExecState* exec, const ArgList& args)
{
    JSSVGPODTypeWrapper<SVGLength>* imp = impl();
    SVGElement* context = JSSVGContextCache::svgContextForDOMObject(this);

    SVGLength podImp(*imp);
    podImp.convertToSpecifiedUnits(args.at(0).toInt32(exec), context);

    imp->commitChange(podImp, this);
    return jsUndefined();
}

}

#endif // ENABLE(SVG)
