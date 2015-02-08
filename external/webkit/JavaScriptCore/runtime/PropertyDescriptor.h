

#ifndef PropertyDescriptor_h
#define PropertyDescriptor_h

#include "JSValue.h"

namespace JSC {
    class PropertyDescriptor {
    public:
        PropertyDescriptor()
            : m_attributes(defaultAttributes)
            , m_seenAttributes(0)
        {
        }
        bool writable() const;
        bool enumerable() const;
        bool configurable() const;
        bool isDataDescriptor() const;
        bool isGenericDescriptor() const;
        bool isAccessorDescriptor() const;
        unsigned attributes() const { return m_attributes; }
        JSValue value() const { return m_value; }
        JSValue getter() const;
        JSValue setter() const;
        void setUndefined();
        void setDescriptor(JSValue value, unsigned attributes);
        void setAccessorDescriptor(JSValue getter, JSValue setter, unsigned attributes);
        void setWritable(bool);
        void setEnumerable(bool);
        void setConfigurable(bool);
        void setValue(JSValue value) { m_value = value; }
        void setSetter(JSValue);
        void setGetter(JSValue);
        bool isEmpty() const { return !(m_value || m_getter || m_setter || m_seenAttributes); }
        bool writablePresent() const { return m_seenAttributes & WritablePresent; }
        bool enumerablePresent() const { return m_seenAttributes & EnumerablePresent; }
        bool configurablePresent() const { return m_seenAttributes & ConfigurablePresent; }
        bool setterPresent() const { return m_setter; }
        bool getterPresent() const { return m_getter; }
        bool equalTo(ExecState* exec, const PropertyDescriptor& other) const;
        bool attributesEqual(const PropertyDescriptor& other) const;
        unsigned attributesWithOverride(const PropertyDescriptor& other) const;
    private:
        static unsigned defaultAttributes;
        bool operator==(const PropertyDescriptor&){ return false; }
        enum { WritablePresent = 1, EnumerablePresent = 2, ConfigurablePresent = 4};
        // May be a getter/setter
        JSValue m_value;
        JSValue m_getter;
        JSValue m_setter;
        unsigned m_attributes;
        unsigned m_seenAttributes;
    };
}

#endif
