

#ifndef UserObjectImp_h
#define UserObjectImp_h

#include "JSUtils.h"
#include "JSBase.h"
#include "JSObject.h"
#include <JavaScriptCore/JSType.h>

class UserObjectImp : public JSObject {
public:
    UserObjectImp(PassRefPtr<Structure>, JSUserObject*);
    virtual ~UserObjectImp();

    virtual const ClassInfo *classInfo() const;
    static const ClassInfo info;

    virtual CallType getCallData(CallData&);

    virtual void getOwnPropertyNames(ExecState*, PropertyNameArray&, EnumerationMode mode = ExcludeDontEnumProperties);

    virtual JSValue callAsFunction(ExecState *exec, JSObject *thisObj, const ArgList &args);
    virtual bool getOwnPropertySlot(ExecState *, const Identifier&, PropertySlot&);
    virtual void put(ExecState *exec, const Identifier &propertyName, JSValue value, PutPropertySlot&);

    JSValue toPrimitive(ExecState*, JSType preferredType = UnspecifiedType) const;
    virtual bool toBoolean(ExecState *exec) const;
    virtual double toNumber(ExecState *exec) const;
    virtual UString toString(ExecState *exec) const;

    virtual void markChildren(MarkStack&);

    JSUserObject *GetJSUserObject() const;

    static PassRefPtr<Structure> createStructure(JSValue prototype)
    {
        return Structure::create(prototype, TypeInfo(ObjectType, OverridesGetOwnPropertySlot | OverridesMarkChildren | OverridesGetPropertyNames), AnonymousSlotCount);
    }

private:
    static JSValue userObjectGetter(ExecState*, const Identifier& propertyName, const PropertySlot&);

    JSUserObject* fJSUserObject;
};

#endif
