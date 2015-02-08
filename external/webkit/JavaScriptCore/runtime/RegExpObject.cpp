

#include "config.h"
#include "RegExpObject.h"

#include "Error.h"
#include "JSArray.h"
#include "JSGlobalObject.h"
#include "JSString.h"
#include "RegExpConstructor.h"
#include "RegExpPrototype.h"

namespace JSC {

static JSValue regExpObjectGlobal(ExecState*, const Identifier&, const PropertySlot&);
static JSValue regExpObjectIgnoreCase(ExecState*, const Identifier&, const PropertySlot&);
static JSValue regExpObjectMultiline(ExecState*, const Identifier&, const PropertySlot&);
static JSValue regExpObjectSource(ExecState*, const Identifier&, const PropertySlot&);
static JSValue regExpObjectLastIndex(ExecState*, const Identifier&, const PropertySlot&);
static void setRegExpObjectLastIndex(ExecState*, JSObject*, JSValue);

} // namespace JSC

#include "RegExpObject.lut.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(RegExpObject);

const ClassInfo RegExpObject::info = { "RegExp", 0, 0, ExecState::regExpTable };


RegExpObject::RegExpObject(NonNullPassRefPtr<Structure> structure, NonNullPassRefPtr<RegExp> regExp)
    : JSObject(structure)
    , d(new RegExpObjectData(regExp, 0))
{
}

RegExpObject::~RegExpObject()
{
}

bool RegExpObject::getOwnPropertySlot(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
{
    return getStaticValueSlot<RegExpObject, JSObject>(exec, ExecState::regExpTable(exec), this, propertyName, slot);
}

bool RegExpObject::getOwnPropertyDescriptor(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    return getStaticValueDescriptor<RegExpObject, JSObject>(exec, ExecState::regExpTable(exec), this, propertyName, descriptor);
}

JSValue regExpObjectGlobal(ExecState*, const Identifier&, const PropertySlot& slot)
{
    return jsBoolean(asRegExpObject(slot.slotBase())->regExp()->global());
}

JSValue regExpObjectIgnoreCase(ExecState*, const Identifier&, const PropertySlot& slot)
{
    return jsBoolean(asRegExpObject(slot.slotBase())->regExp()->ignoreCase());
}
 
JSValue regExpObjectMultiline(ExecState*, const Identifier&, const PropertySlot& slot)
{            
    return jsBoolean(asRegExpObject(slot.slotBase())->regExp()->multiline());
}

JSValue regExpObjectSource(ExecState* exec, const Identifier&, const PropertySlot& slot)
{
    return jsString(exec, asRegExpObject(slot.slotBase())->regExp()->pattern());
}

JSValue regExpObjectLastIndex(ExecState* exec, const Identifier&, const PropertySlot& slot)
{
    return jsNumber(exec, asRegExpObject(slot.slotBase())->lastIndex());
}

void RegExpObject::put(ExecState* exec, const Identifier& propertyName, JSValue value, PutPropertySlot& slot)
{
    lookupPut<RegExpObject, JSObject>(exec, propertyName, value, ExecState::regExpTable(exec), this, slot);
}

void setRegExpObjectLastIndex(ExecState* exec, JSObject* baseObject, JSValue value)
{
    asRegExpObject(baseObject)->setLastIndex(value.toInteger(exec));
}

JSValue RegExpObject::test(ExecState* exec, const ArgList& args)
{
    return jsBoolean(match(exec, args));
}

JSValue RegExpObject::exec(ExecState* exec, const ArgList& args)
{
    if (match(exec, args))
        return exec->lexicalGlobalObject()->regExpConstructor()->arrayOfMatches(exec);
    return jsNull();
}

static JSValue JSC_HOST_CALL callRegExpObject(ExecState* exec, JSObject* function, JSValue, const ArgList& args)
{
    return asRegExpObject(function)->exec(exec, args);
}

CallType RegExpObject::getCallData(CallData& callData)
{
    callData.native.function = callRegExpObject;
    return CallTypeHost;
}

// Shared implementation used by test and exec.
bool RegExpObject::match(ExecState* exec, const ArgList& args)
{
    RegExpConstructor* regExpConstructor = exec->lexicalGlobalObject()->regExpConstructor();

    UString input = args.isEmpty() ? regExpConstructor->input() : args.at(0).toString(exec);
    if (input.isNull()) {
        throwError(exec, GeneralError, makeString("No input to ", toString(exec), "."));
        return false;
    }

    if (!regExp()->global()) {
        int position;
        int length;
        regExpConstructor->performMatch(d->regExp.get(), input, 0, position, length);
        return position >= 0;
    }

    if (d->lastIndex < 0 || d->lastIndex > input.size()) {
        d->lastIndex = 0;
        return false;
    }

    int position;
    int length = 0;
    regExpConstructor->performMatch(d->regExp.get(), input, static_cast<int>(d->lastIndex), position, length);
    if (position < 0) {
        d->lastIndex = 0;
        return false;
    }

    d->lastIndex = position + length;
    return true;
}

} // namespace JSC
