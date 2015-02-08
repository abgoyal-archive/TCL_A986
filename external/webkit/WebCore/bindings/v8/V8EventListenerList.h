

#ifndef V8EventListenerList_h
#define V8EventListenerList_h

#include "V8CustomEventListener.h"
#include "V8HiddenPropertyName.h"

#include <v8.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {
    class Frame;

    // This is a container for V8EventListener objects that uses hidden properties of v8::Object to speed up lookups.
    class V8EventListenerList {
    public:
        static PassRefPtr<V8EventListener> findWrapper(v8::Local<v8::Value> value, bool isAttribute)
        {
            ASSERT(v8::Context::InContext());
            if (!value->IsObject())
                return 0;

            v8::Handle<v8::String> wrapperProperty = getHiddenProperty(isAttribute);
            return doFindWrapper(v8::Local<v8::Object>::Cast(value), wrapperProperty);
        }

        template<typename WrapperType>
        static PassRefPtr<V8EventListener> findOrCreateWrapper(v8::Local<v8::Value>, bool isAttribute);

        static void clearWrapper(v8::Handle<v8::Object> listenerObject, bool isAttribute)
        {
            v8::Handle<v8::String> wrapperProperty = getHiddenProperty(isAttribute);
            listenerObject->DeleteHiddenValue(wrapperProperty);
        }

    private:
        static V8EventListener* doFindWrapper(v8::Local<v8::Object> object, v8::Handle<v8::String> wrapperProperty)
        {
            ASSERT(v8::Context::InContext());
            v8::HandleScope scope;
            v8::Local<v8::Value> listener = object->GetHiddenValue(wrapperProperty);
            if (listener.IsEmpty())
                return 0;
            return static_cast<V8EventListener*>(v8::External::Unwrap(listener));
        }

        static inline v8::Handle<v8::String> getHiddenProperty(bool isAttribute)
        {
            return isAttribute ? V8HiddenPropertyName::attributeListener() : V8HiddenPropertyName::listener();
        }
    };

    template<typename WrapperType>
    PassRefPtr<V8EventListener> V8EventListenerList::findOrCreateWrapper(v8::Local<v8::Value> value, bool isAttribute)
    {
        ASSERT(v8::Context::InContext());
        if (!value->IsObject())
            return 0;

        v8::Local<v8::Object> object = v8::Local<v8::Object>::Cast(value);
        v8::Handle<v8::String> wrapperProperty = getHiddenProperty(isAttribute);

        V8EventListener* wrapper = doFindWrapper(object, wrapperProperty);
        if (wrapper)
            return wrapper;

        PassRefPtr<V8EventListener> wrapperPtr = WrapperType::create(object, isAttribute, WorldContextHandle(UseCurrentWorld));
        if (wrapperPtr)
            object->SetHiddenValue(wrapperProperty, v8::External::Wrap(wrapperPtr.get()));

        return wrapperPtr;
    }

} // namespace WebCore

#endif // V8EventListenerList_h
