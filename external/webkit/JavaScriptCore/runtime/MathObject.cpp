

#include "config.h"
#include "MathObject.h"

#include "ObjectPrototype.h"
#include "Operations.h"
#include <time.h>
#include <wtf/Assertions.h>
#include <wtf/MathExtras.h>
#include <wtf/RandomNumber.h>
#include <wtf/RandomNumberSeed.h>

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(MathObject);

static JSValue JSC_HOST_CALL mathProtoFuncAbs(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncACos(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncASin(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncATan(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncATan2(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncCeil(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncCos(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncExp(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncFloor(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncLog(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncMax(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncMin(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncPow(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncRandom(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncRound(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncSin(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncSqrt(ExecState*, JSObject*, JSValue, const ArgList&);
static JSValue JSC_HOST_CALL mathProtoFuncTan(ExecState*, JSObject*, JSValue, const ArgList&);

}

#include "MathObject.lut.h"

namespace JSC {

// ------------------------------ MathObject --------------------------------

const ClassInfo MathObject::info = { "Math", 0, 0, ExecState::mathTable };


MathObject::MathObject(ExecState* exec, NonNullPassRefPtr<Structure> structure)
    : JSObject(structure)
{
    putDirectWithoutTransition(Identifier(exec, "E"), jsNumber(exec, exp(1.0)), DontDelete | DontEnum | ReadOnly);
    putDirectWithoutTransition(Identifier(exec, "LN2"), jsNumber(exec, log(2.0)), DontDelete | DontEnum | ReadOnly);
    putDirectWithoutTransition(Identifier(exec, "LN10"), jsNumber(exec, log(10.0)), DontDelete | DontEnum | ReadOnly);
    putDirectWithoutTransition(Identifier(exec, "LOG2E"), jsNumber(exec, 1.0 / log(2.0)), DontDelete | DontEnum | ReadOnly);
    putDirectWithoutTransition(Identifier(exec, "LOG10E"), jsNumber(exec, 1.0 / log(10.0)), DontDelete | DontEnum | ReadOnly);
    putDirectWithoutTransition(Identifier(exec, "PI"), jsNumber(exec, piDouble), DontDelete | DontEnum | ReadOnly);
    putDirectWithoutTransition(Identifier(exec, "SQRT1_2"), jsNumber(exec, sqrt(0.5)), DontDelete | DontEnum | ReadOnly);
    putDirectWithoutTransition(Identifier(exec, "SQRT2"), jsNumber(exec, sqrt(2.0)), DontDelete | DontEnum | ReadOnly);
}

// ECMA 15.8

bool MathObject::getOwnPropertySlot(ExecState* exec, const Identifier& propertyName, PropertySlot &slot)
{
    return getStaticFunctionSlot<JSObject>(exec, ExecState::mathTable(exec), this, propertyName, slot);
}

bool MathObject::getOwnPropertyDescriptor(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    return getStaticFunctionDescriptor<JSObject>(exec, ExecState::mathTable(exec), this, propertyName, descriptor);
}

// ------------------------------ Functions --------------------------------

JSValue JSC_HOST_CALL mathProtoFuncAbs(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsNumber(exec, fabs(args.at(0).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncACos(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsDoubleNumber(exec, acos(args.at(0).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncASin(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsDoubleNumber(exec, asin(args.at(0).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncATan(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsDoubleNumber(exec, atan(args.at(0).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncATan2(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsDoubleNumber(exec, atan2(args.at(0).toNumber(exec), args.at(1).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncCeil(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsNumber(exec, ceil(args.at(0).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncCos(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsDoubleNumber(exec, cos(args.at(0).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncExp(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsDoubleNumber(exec, exp(args.at(0).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncFloor(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsNumber(exec, floor(args.at(0).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncLog(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsDoubleNumber(exec, log(args.at(0).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncMax(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    unsigned argsCount = args.size();
    double result = -Inf;
    for (unsigned k = 0; k < argsCount; ++k) {
        double val = args.at(k).toNumber(exec);
        if (isnan(val)) {
            result = NaN;
            break;
        }
        if (val > result || (val == 0 && result == 0 && !signbit(val)))
            result = val;
    }
    return jsNumber(exec, result);
}

JSValue JSC_HOST_CALL mathProtoFuncMin(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    unsigned argsCount = args.size();
    double result = +Inf;
    for (unsigned k = 0; k < argsCount; ++k) {
        double val = args.at(k).toNumber(exec);
        if (isnan(val)) {
            result = NaN;
            break;
        }
        if (val < result || (val == 0 && result == 0 && signbit(val)))
            result = val;
    }
    return jsNumber(exec, result);
}

JSValue JSC_HOST_CALL mathProtoFuncPow(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    // ECMA 15.8.2.1.13

    double arg = args.at(0).toNumber(exec);
    double arg2 = args.at(1).toNumber(exec);

    if (isnan(arg2))
        return jsNaN(exec);
    if (isinf(arg2) && fabs(arg) == 1)
        return jsNaN(exec);
    return jsNumber(exec, pow(arg, arg2));
}

JSValue JSC_HOST_CALL mathProtoFuncRandom(ExecState* exec, JSObject*, JSValue, const ArgList&)
{
    return jsDoubleNumber(exec, exec->globalData().weakRandom.get());
}

JSValue JSC_HOST_CALL mathProtoFuncRound(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    double arg = args.at(0).toNumber(exec);
    if (signbit(arg) && arg >= -0.5)
         return jsNumber(exec, -0.0);
    double integer = ceil(arg);
    return jsNumber(exec, integer - (integer - arg > 0.5));
}

JSValue JSC_HOST_CALL mathProtoFuncSin(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsDoubleNumber(exec, sin(args.at(0).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncSqrt(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsDoubleNumber(exec, sqrt(args.at(0).toNumber(exec)));
}

JSValue JSC_HOST_CALL mathProtoFuncTan(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsDoubleNumber(exec, tan(args.at(0).toNumber(exec)));
}

} // namespace JSC
