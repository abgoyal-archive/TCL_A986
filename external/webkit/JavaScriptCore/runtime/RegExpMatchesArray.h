

#ifndef RegExpMatchesArray_h
#define RegExpMatchesArray_h

#include "JSArray.h"

namespace JSC {

    class RegExpMatchesArray : public JSArray {
    public:
        RegExpMatchesArray(ExecState*, RegExpConstructorPrivate*);
        virtual ~RegExpMatchesArray();

    private:
        virtual bool getOwnPropertySlot(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
        {
            if (lazyCreationData())
                fillArrayInstance(exec);
            return JSArray::getOwnPropertySlot(exec, propertyName, slot);
        }

        virtual bool getOwnPropertySlot(ExecState* exec, unsigned propertyName, PropertySlot& slot)
        {
            if (lazyCreationData())
                fillArrayInstance(exec);
            return JSArray::getOwnPropertySlot(exec, propertyName, slot);
        }

        virtual bool getOwnPropertyDescriptor(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
        {
            if (lazyCreationData())
                fillArrayInstance(exec);
            return JSArray::getOwnPropertyDescriptor(exec, propertyName, descriptor);
        }

        virtual void put(ExecState* exec, const Identifier& propertyName, JSValue v, PutPropertySlot& slot)
        {
            if (lazyCreationData())
                fillArrayInstance(exec);
            JSArray::put(exec, propertyName, v, slot);
        }

        virtual void put(ExecState* exec, unsigned propertyName, JSValue v)
        {
            if (lazyCreationData())
                fillArrayInstance(exec);
            JSArray::put(exec, propertyName, v);
        }

        virtual bool deleteProperty(ExecState* exec, const Identifier& propertyName)
        {
            if (lazyCreationData())
                fillArrayInstance(exec);
            return JSArray::deleteProperty(exec, propertyName);
        }

        virtual bool deleteProperty(ExecState* exec, unsigned propertyName)
        {
            if (lazyCreationData())
                fillArrayInstance(exec);
            return JSArray::deleteProperty(exec, propertyName);
        }

        virtual void getOwnPropertyNames(ExecState* exec, PropertyNameArray& arr, EnumerationMode mode = ExcludeDontEnumProperties)
        {
            if (lazyCreationData())
                fillArrayInstance(exec);
            JSArray::getOwnPropertyNames(exec, arr, mode);
        }

        void fillArrayInstance(ExecState*);
};

}

#endif // RegExpMatchesArray_h
