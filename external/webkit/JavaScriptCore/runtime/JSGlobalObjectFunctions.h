

#ifndef JSGlobalObjectFunctions_h
#define JSGlobalObjectFunctions_h

#include <wtf/unicode/Unicode.h>

namespace JSC {

    class ArgList;
    class ExecState;
    class JSObject;
    class JSValue;

    // FIXME: These functions should really be in JSGlobalObject.cpp, but putting them there
    // is a 0.5% reduction.

    JSValue JSC_HOST_CALL globalFuncEval(ExecState*, JSObject*, JSValue, const ArgList&);
    JSValue JSC_HOST_CALL globalFuncParseInt(ExecState*, JSObject*, JSValue, const ArgList&);
    JSValue JSC_HOST_CALL globalFuncParseFloat(ExecState*, JSObject*, JSValue, const ArgList&);
    JSValue JSC_HOST_CALL globalFuncIsNaN(ExecState*, JSObject*, JSValue, const ArgList&);
    JSValue JSC_HOST_CALL globalFuncIsFinite(ExecState*, JSObject*, JSValue, const ArgList&);
    JSValue JSC_HOST_CALL globalFuncDecodeURI(ExecState*, JSObject*, JSValue, const ArgList&);
    JSValue JSC_HOST_CALL globalFuncDecodeURIComponent(ExecState*, JSObject*, JSValue, const ArgList&);
    JSValue JSC_HOST_CALL globalFuncEncodeURI(ExecState*, JSObject*, JSValue, const ArgList&);
    JSValue JSC_HOST_CALL globalFuncEncodeURIComponent(ExecState*, JSObject*, JSValue, const ArgList&);
    JSValue JSC_HOST_CALL globalFuncEscape(ExecState*, JSObject*, JSValue, const ArgList&);
    JSValue JSC_HOST_CALL globalFuncUnescape(ExecState*, JSObject*, JSValue, const ArgList&);
#ifndef NDEBUG
    JSValue JSC_HOST_CALL globalFuncJSCPrint(ExecState*, JSObject*, JSValue, const ArgList&);
#endif

    static const double mantissaOverflowLowerBound = 9007199254740992.0;
    double parseIntOverflow(const char*, int length, int radix);
    bool isStrWhiteSpace(UChar);

} // namespace JSC

#endif // JSGlobalObjectFunctions_h
