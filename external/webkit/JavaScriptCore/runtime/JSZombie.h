

#ifndef JSZombie_h
#define JSZombie_h

#include "JSCell.h"

#if ENABLE(JSC_ZOMBIES)
namespace JSC {

class JSZombie : public JSCell {
public:
    JSZombie(const ClassInfo* oldInfo, Structure* structure)
        : JSCell(structure)
        , m_oldInfo(oldInfo)
    {
    }
    virtual bool isZombie() const { return true; }
    virtual const ClassInfo* classInfo() const { return &s_info; }
    static Structure* leakedZombieStructure();

    virtual bool isGetterSetter() const { ASSERT_NOT_REACHED(); return false; }
    virtual bool isAPIValueWrapper() const { ASSERT_NOT_REACHED(); return false; }
    virtual bool isPropertyNameIterator() const { ASSERT_NOT_REACHED(); return false; }
    virtual CallType getCallData(CallData&) { ASSERT_NOT_REACHED(); return CallTypeNone; }
    virtual ConstructType getConstructData(ConstructData&) { ASSERT_NOT_REACHED(); return ConstructTypeNone; }
    virtual bool getUInt32(uint32_t&) const { ASSERT_NOT_REACHED(); return false; }
    virtual JSValue toPrimitive(ExecState*, PreferredPrimitiveType) const { ASSERT_NOT_REACHED(); return jsNull(); }
    virtual bool getPrimitiveNumber(ExecState*, double&, JSValue&) { ASSERT_NOT_REACHED(); return false; }
    virtual bool toBoolean(ExecState*) const { ASSERT_NOT_REACHED(); return false; }
    virtual double toNumber(ExecState*) const { ASSERT_NOT_REACHED(); return 0.0; }
    virtual UString toString(ExecState*) const { ASSERT_NOT_REACHED(); return ""; }
    virtual JSObject* toObject(ExecState*) const { ASSERT_NOT_REACHED(); return 0; }
    virtual void markChildren(MarkStack&) { ASSERT_NOT_REACHED(); }
    virtual void put(ExecState*, const Identifier&, JSValue, PutPropertySlot&) { ASSERT_NOT_REACHED(); }
    virtual void put(ExecState*, unsigned, JSValue) { ASSERT_NOT_REACHED(); }
    virtual bool deleteProperty(ExecState*, const Identifier&) { ASSERT_NOT_REACHED(); return false; }
    virtual bool deleteProperty(ExecState*, unsigned) { ASSERT_NOT_REACHED(); return false; }
    virtual JSObject* toThisObject(ExecState*) const { ASSERT_NOT_REACHED(); return 0; }
    virtual UString toThisString(ExecState*) const { ASSERT_NOT_REACHED(); return ""; }
    virtual JSString* toThisJSString(ExecState*) { ASSERT_NOT_REACHED(); return 0; }
    virtual JSValue getJSNumber() { ASSERT_NOT_REACHED(); return jsNull(); }
    virtual bool getOwnPropertySlot(ExecState*, const Identifier&, PropertySlot&) { ASSERT_NOT_REACHED(); return false; }
    virtual bool getOwnPropertySlot(ExecState*, unsigned, PropertySlot&) { ASSERT_NOT_REACHED(); return false; }
    
    static const ClassInfo s_info;
private:
    const ClassInfo* m_oldInfo;
};

}

#endif // ENABLE(JSC_ZOMBIES)

#endif // JSZombie_h
