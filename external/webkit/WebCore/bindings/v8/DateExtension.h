

#ifndef DateExtension_h
#define DateExtension_h

#include <v8.h>

namespace WebCore {

// Prevent "sleep" calls in unload handlers.
class DateExtension : public v8::Extension {
public:
    static DateExtension* get();
    void setAllowSleep(bool allow);

private:
    DateExtension();
    virtual v8::Handle<v8::FunctionTemplate> GetNativeFunction(v8::Handle<v8::String>);
    static v8::Handle<v8::Value> Setup(const v8::Arguments&);
    static v8::Handle<v8::Value> OnSleepDetected(const v8::Arguments&);

    static DateExtension* extension;
};

} // namespace WebCore

#endif  // DateExtension_h
