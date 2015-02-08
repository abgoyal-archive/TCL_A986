

#ifndef Error_h
#define Error_h

#include <stdint.h>

namespace JSC {

    class ExecState;
    class JSObject;
    class UString;

    /**
     * Types of Native Errors available. For custom errors, GeneralError
     * should be used.
     */
    enum ErrorType {
        GeneralError   = 0,
        EvalError      = 1,
        RangeError     = 2,
        ReferenceError = 3,
        SyntaxError    = 4,
        TypeError      = 5,
        URIError       = 6
    };
    
    extern const char* expressionBeginOffsetPropertyName;
    extern const char* expressionCaretOffsetPropertyName;
    extern const char* expressionEndOffsetPropertyName;
    
    class Error {
    public:
        static JSObject* create(ExecState*, ErrorType, const UString& message, int lineNumber, intptr_t sourceID, const UString& sourceURL);
        static JSObject* create(ExecState*, ErrorType, const char* message);
    };

    JSObject* throwError(ExecState*, ErrorType, const UString& message, int lineNumber, intptr_t sourceID, const UString& sourceURL);
    JSObject* throwError(ExecState*, ErrorType, const UString& message);
    JSObject* throwError(ExecState*, ErrorType, const char* message);
    JSObject* throwError(ExecState*, ErrorType);
    JSObject* throwError(ExecState*, JSObject*);

} // namespace JSC

#endif // Error_h
