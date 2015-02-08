

#ifndef GCController_h
#define GCController_h

#include <JavaScriptCore/JSObjectRef.h>

class GCController {
public:
    GCController();
    ~GCController();

    void makeWindowObject(JSContextRef context, JSObjectRef windowObject, JSValueRef* exception);

    // Controller Methods - platfrom independant implementations
    void collect() const;
    void collectOnAlternateThread(bool waitUntilDone) const;
    size_t getJSObjectCount() const;

private:
    static JSClassRef getJSClass();
};

#endif // GCController_h
