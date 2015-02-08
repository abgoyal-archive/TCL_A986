

#ifndef JSByteArray_h
#define JSByteArray_h

#include "JSObject.h"

#include <wtf/ByteArray.h>

namespace JSC {

    class JSByteArray : public JSObject {
        friend class JSGlobalData;
    public:
        bool canAccessIndex(unsigned i) { return i < m_storage->length(); }
        JSValue getIndex(ExecState* exec, unsigned i)
        {
            ASSERT(canAccessIndex(i));
            return jsNumber(exec, m_storage->data()[i]);
        }

        void setIndex(unsigned i, int value)
        {
            ASSERT(canAccessIndex(i));
            if (value & ~0xFF) {
                if (value < 0)
                    value = 0;
                else
                    value = 255;
            }
            m_storage->data()[i] = static_cast<unsigned char>(value);
        }
        
        void setIndex(unsigned i, double value)
        {
            ASSERT(canAccessIndex(i));
            if (!(value > 0)) // Clamp NaN to 0
                value = 0;
            else if (value > 255)
                value = 255;
            m_storage->data()[i] = static_cast<unsigned char>(value + 0.5);
        }
        
        void setIndex(ExecState* exec, unsigned i, JSValue value)
        {
            double byteValue = value.toNumber(exec);
            if (exec->hadException())
                return;
            if (canAccessIndex(i))
                setIndex(i, byteValue);
        }

        JSByteArray(ExecState* exec, NonNullPassRefPtr<Structure>, WTF::ByteArray* storage, const JSC::ClassInfo* = &s_defaultInfo);
        static PassRefPtr<Structure> createStructure(JSValue prototype);
        
        virtual bool getOwnPropertySlot(JSC::ExecState*, const JSC::Identifier& propertyName, JSC::PropertySlot&);
        virtual bool getOwnPropertySlot(JSC::ExecState*, unsigned propertyName, JSC::PropertySlot&);
        virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);
        virtual void put(JSC::ExecState*, const JSC::Identifier& propertyName, JSC::JSValue, JSC::PutPropertySlot&);
        virtual void put(JSC::ExecState*, unsigned propertyName, JSC::JSValue);

        virtual void getOwnPropertyNames(JSC::ExecState*, JSC::PropertyNameArray&, EnumerationMode mode = ExcludeDontEnumProperties);

        virtual const ClassInfo* classInfo() const { return m_classInfo; }
        static const ClassInfo s_defaultInfo;
        
        size_t length() const { return m_storage->length(); }

        WTF::ByteArray* storage() const { return m_storage.get(); }

#if !ASSERT_DISABLED
        virtual ~JSByteArray();
#endif

    protected:
        static const unsigned StructureFlags = OverridesGetOwnPropertySlot | OverridesGetPropertyNames | JSObject::StructureFlags;

    private:
        enum VPtrStealingHackType { VPtrStealingHack };
        JSByteArray(VPtrStealingHackType) 
            : JSObject(createStructure(jsNull()))
            , m_classInfo(0)
        {
        }

        RefPtr<WTF::ByteArray> m_storage;
        const ClassInfo* m_classInfo;
    };
    
    JSByteArray* asByteArray(JSValue value);
    inline JSByteArray* asByteArray(JSValue value)
    {
        return static_cast<JSByteArray*>(asCell(value));
    }

    inline bool isJSByteArray(JSGlobalData* globalData, JSValue v) { return v.isCell() && v.asCell()->vptr() == globalData->jsByteArrayVPtr; }

} // namespace JSC

#endif // JSByteArray_h
