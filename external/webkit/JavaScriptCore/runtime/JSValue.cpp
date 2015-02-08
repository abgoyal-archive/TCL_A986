

#include "config.h"
#include "JSValue.h"

#include "BooleanConstructor.h"
#include "BooleanPrototype.h"
#include "ExceptionHelpers.h"
#include "JSGlobalObject.h"
#include "JSFunction.h"
#include "JSNotAnObject.h"
#include "NumberObject.h"
#include <wtf/MathExtras.h>
#include <wtf/StringExtras.h>

namespace JSC {

static const double D32 = 4294967296.0;

// ECMA 9.4
double JSValue::toInteger(ExecState* exec) const
{
    if (isInt32())
        return asInt32();
    double d = toNumber(exec);
    return isnan(d) ? 0.0 : trunc(d);
}

double JSValue::toIntegerPreserveNaN(ExecState* exec) const
{
    if (isInt32())
        return asInt32();
    return trunc(toNumber(exec));
}

JSObject* JSValue::toObjectSlowCase(ExecState* exec) const
{
    ASSERT(!isCell());

    if (isInt32() || isDouble())
        return constructNumber(exec, asValue());
    if (isTrue() || isFalse())
        return constructBooleanFromImmediateBoolean(exec, asValue());
    ASSERT(isUndefinedOrNull());
    JSNotAnObjectErrorStub* exception = createNotAnObjectErrorStub(exec, isNull());
    exec->setException(exception);
    return new (exec) JSNotAnObject(exec, exception);
}

JSObject* JSValue::toThisObjectSlowCase(ExecState* exec) const
{
    ASSERT(!isCell());

    if (isInt32() || isDouble())
        return constructNumber(exec, asValue());
    if (isTrue() || isFalse())
        return constructBooleanFromImmediateBoolean(exec, asValue());
    ASSERT(isUndefinedOrNull());
    return exec->globalThisValue();
}

JSObject* JSValue::synthesizeObject(ExecState* exec) const
{
    ASSERT(!isCell());
    if (isNumber())
        return constructNumber(exec, asValue());
    if (isBoolean())
        return constructBooleanFromImmediateBoolean(exec, asValue());
    
    JSNotAnObjectErrorStub* exception = createNotAnObjectErrorStub(exec, isNull());
    exec->setException(exception);
    return new (exec) JSNotAnObject(exec, exception);
}

JSObject* JSValue::synthesizePrototype(ExecState* exec) const
{
    ASSERT(!isCell());
    if (isNumber())
        return exec->lexicalGlobalObject()->numberPrototype();
    if (isBoolean())
        return exec->lexicalGlobalObject()->booleanPrototype();

    JSNotAnObjectErrorStub* exception = createNotAnObjectErrorStub(exec, isNull());
    exec->setException(exception);
    return new (exec) JSNotAnObject(exec, exception);
}

#ifndef NDEBUG
char* JSValue::description()
{
    static const size_t size = 32;
    static char description[size];

    if (!*this)
        snprintf(description, size, "<JSValue()>");
    else if (isInt32())
        snprintf(description, size, "Int32: %d", asInt32());
    else if (isDouble())
        snprintf(description, size, "Double: %lf", asDouble());
    else if (isCell())
        snprintf(description, size, "Cell: %p", asCell());
    else if (isTrue())
        snprintf(description, size, "True");
    else if (isFalse())
        snprintf(description, size, "False");
    else if (isNull())
        snprintf(description, size, "Null");
    else {
        ASSERT(isUndefined());
        snprintf(description, size, "Undefined");
    }

    return description;
}
#endif

int32_t toInt32SlowCase(double d, bool& ok)
{
    ok = true;

    if (d >= -D32 / 2 && d < D32 / 2)
        return static_cast<int32_t>(d);

    if (isnan(d) || isinf(d)) {
        ok = false;
        return 0;
    }

    double d32 = fmod(trunc(d), D32);
    if (d32 >= D32 / 2)
        d32 -= D32;
    else if (d32 < -D32 / 2)
        d32 += D32;
    return static_cast<int32_t>(d32);
}

uint32_t toUInt32SlowCase(double d, bool& ok)
{
    ok = true;

    if (d >= 0.0 && d < D32)
        return static_cast<uint32_t>(d);

    if (isnan(d) || isinf(d)) {
        ok = false;
        return 0;
    }

    double d32 = fmod(trunc(d), D32);
    if (d32 < 0)
        d32 += D32;
    return static_cast<uint32_t>(d32);
}

NEVER_INLINE double nonInlineNaN()
{
#if OS(SYMBIAN)
    return nanval();
#else
    return std::numeric_limits<double>::quiet_NaN();
#endif
}

} // namespace JSC
