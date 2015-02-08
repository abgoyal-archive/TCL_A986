

#ifndef JSUtils_h
#define JSUtils_h

#include "JavaScriptGlue.h"

#include <JavaScriptCore/Collector.h>
#include <JavaScriptCore/JSValue.h>
#include <JavaScriptCore/Completion.h>
#include <JavaScriptCore/Identifier.h>
#include <JavaScriptCore/JSLock.h>
#include <JavaScriptCore/JSObject.h>
#include <JavaScriptCore/JSGlobalObject.h>
#include <JavaScriptCore/Protect.h>
#include <JavaScriptCore/UString.h>

#include <wtf/RefPtr.h>

using namespace JSC;

class JSBase;
class JSUserObject;
class JSRun;
class JSValueWrapper;
class JSUserObjectImp;

UString CFStringToUString(CFStringRef inCFString);
CFStringRef UStringToCFString(const UString& inUString);
Identifier CFStringToIdentifier(CFStringRef inCFString, ExecState*);
CFStringRef IdentifierToCFString(const Identifier& inIdentifier);
JSUserObject *KJSValueToJSObject(JSValue inValue, ExecState *exec);
CFTypeRef KJSValueToCFType(JSValue inValue, ExecState *exec);
JSValue JSObjectKJSValue(JSUserObject* ptr);
CFTypeRef GetCFNull(void);

inline CFTypeRef RetainCFType(CFTypeRef x) { if (x) x = CFRetain(x); return x; }
inline void ReleaseCFType(CFTypeRef x) { if (x) CFRelease(x);  }

ExecState* getThreadGlobalExecState();

enum {
    kJSInvalidTypeID = 0,
    kJSObjectTypeID,
    kJSRunTypeID
};

enum {
    kJSUserObjectDataTypeUnknown,
    kJSUserObjectDataTypeJSValueWrapper,
    kJSUserObjectDataTypeCFType
};

class JSGlueAPIEntry {
public:
    JSGlueAPIEntry();
    ~JSGlueAPIEntry();

private:
    JSLock m_lock;
    IdentifierTable* m_storedIdentifierTable;
};

class JSGlueAPICallback {
public:
    JSGlueAPICallback(ExecState*);
    ~JSGlueAPICallback();

private:
    JSLock::DropAllLocks m_dropLocks;
};

#endif
