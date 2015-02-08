

#include "config.h"
#include "JSCoordinates.h"

#include "Coordinates.h"

using namespace JSC;

namespace WebCore {
    
JSValue JSCoordinates::altitude(ExecState* exec) const
{
    Coordinates* imp = impl();
    if (!imp->canProvideAltitude())
        return jsNull();
    return jsNumber(exec, imp->altitude());
}

JSValue JSCoordinates::altitudeAccuracy(ExecState* exec) const
{
    Coordinates* imp = impl();
    if (!imp->canProvideAltitudeAccuracy())
        return jsNull();
    return jsNumber(exec, imp->altitudeAccuracy());
}

JSValue JSCoordinates::heading(ExecState* exec) const
{
    Coordinates* imp = impl();
    if (!imp->canProvideHeading())
        return jsNull();
    return jsNumber(exec, imp->heading());
}

JSValue JSCoordinates::speed(ExecState* exec) const
{
    Coordinates* imp = impl();
    if (!imp->canProvideSpeed())
        return jsNull();
    return jsNumber(exec, imp->speed());
}

} // namespace WebCore
