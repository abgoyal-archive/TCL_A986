

#include "config.h"
#include "DateExtension.h"

#include "V8Proxy.h"
#include "V8HiddenPropertyName.h"

namespace WebCore {

DateExtension* DateExtension::extension;

static const char* dateExtensionName = "v8/DateExtension";
static const char* dateExtensionScript =
    "(function () {"
    "  var counter;"
    "  var orig_getTime;"
    "  function getTimeOverride() {"
    "    if (++counter > 1000)"
    "      OnSleepDetected();"
    "    return orig_getTime.call(this);"
    "  };"
    "  function enableSleepDetection(enable) {"
    "    if (enable) {"
    "      counter = 0;"
    "      orig_getTime = Date.prototype.getTime;"
    "      Date.prototype.getTime = getTimeOverride;"
    "    } else {"
    "      Date.prototype.getTime = orig_getTime;"
    "    }"
    "  };"
    "  native function Setup();"
    "  native function OnSleepDetected();"
    "  Setup(Date, enableSleepDetection);"
    "})()";

DateExtension::DateExtension() : v8::Extension(dateExtensionName, dateExtensionScript)
{
}

DateExtension* DateExtension::get()
{
    if (!extension)
        extension = new DateExtension();
    return extension;
}

void DateExtension::setAllowSleep(bool allow)
{
    v8::Local<v8::Value> result = V8Proxy::currentContext()->Global()->Get(v8::String::New("Date"));
    if (result.IsEmpty())
        return;

    v8::Handle<v8::Object> dateObject = v8::Handle<v8::Object>::Cast(result);
    if (dateObject.IsEmpty())
        return;

    v8::Local<v8::Value> sleepFunctionHandle = dateObject->GetHiddenValue(V8HiddenPropertyName::sleepFunction());
    if (sleepFunctionHandle.IsEmpty() || !sleepFunctionHandle->IsFunction())
        return;

    v8::Handle<v8::Value> argv[1];
    argv[0] = v8::String::New(allow ? "false" : "true");
    v8::Handle<v8::Function>::Cast(sleepFunctionHandle)->Call(v8::Object::New(), 1, argv);
}

v8::Handle<v8::FunctionTemplate> DateExtension::GetNativeFunction(v8::Handle<v8::String> name)
{
    if (name->Equals(v8::String::New("Setup")))
        return v8::FunctionTemplate::New(Setup);
    if (name->Equals(v8::String::New("OnSleepDetected")))
        return v8::FunctionTemplate::New(OnSleepDetected);

    return v8::Handle<v8::FunctionTemplate>();
}

v8::Handle<v8::Value> DateExtension::Setup(const v8::Arguments& args)
{
    if (args.Length() != 2 || !args[0]->IsObject() || !args[1]->IsFunction())
        return v8::Undefined();

    v8::Handle<v8::Object> dateObject = v8::Handle<v8::Object>::Cast(args[0]);
    v8::Handle<v8::Function> enableSleepDetectionFunction = v8::Handle<v8::Function>::Cast(args[1]);

    dateObject->SetHiddenValue(V8HiddenPropertyName::sleepFunction(), enableSleepDetectionFunction);
    return v8::Undefined();
}

v8::Handle<v8::Value> DateExtension::OnSleepDetected(const v8::Arguments&)
{
    V8Proxy::throwError(V8Proxy::GeneralError, "Too much time spent in unload handler.");
    return v8::Undefined();
}

}  // namespace WebCore
