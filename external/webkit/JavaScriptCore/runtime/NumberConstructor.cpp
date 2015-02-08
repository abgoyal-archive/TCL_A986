

#include "config.h"
#include "NumberConstructor.h"

#include "NumberObject.h"
#include "NumberPrototype.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(NumberConstructor);

static JSValue numberConstructorNaNValue(ExecState*, const Identifier&, const PropertySlot&);
static JSValue numberConstructorNegInfinity(ExecState*, const Identifier&, const PropertySlot&);
static JSValue numberConstructorPosInfinity(ExecState*, const Identifier&, const PropertySlot&);
static JSValue numberConstructorMaxValue(ExecState*, const Identifier&, const PropertySlot&);
static JSValue numberConstructorMinValue(ExecState*, const Identifier&, const PropertySlot&);

} // namespace JSC

#include "NumberConstructor.lut.h"

namespace JSC {

const ClassInfo NumberConstructor::info = { "Function", &InternalFunction::info, 0, ExecState::numberTable };


NumberConstructor::NumberConstructor(ExecState* exec, NonNullPassRefPtr<Structure> structure, NumberPrototype* numberPrototype)
    : InternalFunction(&exec->globalData(), structure, Identifier(exec, numberPrototype->info.className))
{
    // Number.Prototype
    putDirectWithoutTransition(exec->propertyNames().prototype, numberPrototype, DontEnum | DontDelete | ReadOnly);

    // no. of arguments for constructor
    putDirectWithoutTransition(exec->propertyNames().length, jsNumber(exec, 1), ReadOnly | DontEnum | DontDelete);
}

bool NumberConstructor::getOwnPropertySlot(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
{
    return getStaticValueSlot<NumberConstructor, InternalFunction>(exec, ExecState::numberTable(exec), this, propertyName, slot);
}

bool NumberConstructor::getOwnPropertyDescriptor(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    return getStaticValueDescriptor<NumberConstructor, InternalFunction>(exec, ExecState::numberTable(exec), this, propertyName, descriptor);
}

static JSValue numberConstructorNaNValue(ExecState* exec, const Identifier&, const PropertySlot&)
{
    return jsNaN(exec);
}

static JSValue numberConstructorNegInfinity(ExecState* exec, const Identifier&, const PropertySlot&)
{
    return jsNumber(exec, -Inf);
}

static JSValue numberConstructorPosInfinity(ExecState* exec, const Identifier&, const PropertySlot&)
{
    return jsNumber(exec, Inf);
}

static JSValue numberConstructorMaxValue(ExecState* exec, const Identifier&, const PropertySlot&)
{
    return jsNumber(exec, 1.7976931348623157E+308);
}

static JSValue numberConstructorMinValue(ExecState* exec, const Identifier&, const PropertySlot&)
{
    return jsNumber(exec, 5E-324);
}

// ECMA 15.7.1
static JSObject* constructWithNumberConstructor(ExecState* exec, JSObject*, const ArgList& args)
{
    NumberObject* object = new (exec) NumberObject(exec->lexicalGlobalObject()->numberObjectStructure());
    double n = args.isEmpty() ? 0 : args.at(0).toNumber(exec);
    object->setInternalValue(jsNumber(exec, n));
    return object;
}

ConstructType NumberConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructWithNumberConstructor;
    return ConstructTypeHost;
}

// ECMA 15.7.2
static JSValue JSC_HOST_CALL callNumberConstructor(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    return jsNumber(exec, args.isEmpty() ? 0 : args.at(0).toNumber(exec));
}

CallType NumberConstructor::getCallData(CallData& callData)
{
    callData.native.function = callNumberConstructor;
    return CallTypeHost;
}

} // namespace JSC
