

#include "config.h"
#include "V8Coordinates.h"

#include "Coordinates.h"
#include "V8Binding.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8Coordinates::altitudeAccessorGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.Coordinates.altitude._get");
    v8::Handle<v8::Object> holder = info.Holder();
    Coordinates* imp = V8Coordinates::toNative(holder);
    if (!imp->canProvideAltitude())
        return v8::Null();
    return v8::Number::New(imp->altitude());
}

v8::Handle<v8::Value> V8Coordinates::altitudeAccuracyAccessorGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.Coordinates.altitudeAccuracy._get");
    v8::Handle<v8::Object> holder = info.Holder();
    Coordinates* imp = V8Coordinates::toNative(holder);
    if (!imp->canProvideAltitudeAccuracy())
        return v8::Null();
    return v8::Number::New(imp->altitudeAccuracy());
}

v8::Handle<v8::Value> V8Coordinates::headingAccessorGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.Coordinates.heading._get");
    v8::Handle<v8::Object> holder = info.Holder();
    Coordinates* imp = V8Coordinates::toNative(holder);
    if (!imp->canProvideHeading())
        return v8::Null();
    return v8::Number::New(imp->heading());
}

v8::Handle<v8::Value> V8Coordinates::speedAccessorGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.Coordinates.speed._get");
    v8::Handle<v8::Object> holder = info.Holder();
    Coordinates* imp = V8Coordinates::toNative(holder);
    if (!imp->canProvideSpeed())
        return v8::Null();
    return v8::Number::New(imp->speed());
}

} // namespace WebCore
