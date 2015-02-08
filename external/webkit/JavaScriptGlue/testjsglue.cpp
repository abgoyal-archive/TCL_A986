

#include "CoreFoundation/CoreFoundation.h"
#include "JavaScriptGlue.h"

CFStringRef script = 
CFSTR("\
x = 1; \n\
function getX() \n\
{ \n\
    return x; \n\
} \n\
");

int main(int argc, char* argv[])
{
    JSRunRef jsRun = JSRunCreate(script, kJSFlagNone);
    if (!JSRunCheckSyntax(jsRun)) {
        return -1;
    }
    JSObjectRef globalObject = JSRunCopyGlobalObject(jsRun);
    JSRunEvaluate(jsRun);
    JSObjectRef getX = JSObjectCopyProperty(globalObject, CFSTR("getX"));
    JSObjectRef jsResult = JSObjectCallFunction(getX, globalObject, 0);

    if (jsResult) {
        CFTypeRef cfResult = JSObjectCopyCFValue(jsResult);
        CFShow(cfResult);
        
        CFRelease(cfResult);
        JSRelease(jsResult);
    }
    
    JSRelease(jsRun);
    
    return 0;
}
