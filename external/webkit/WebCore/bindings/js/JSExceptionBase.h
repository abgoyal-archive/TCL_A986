

#ifndef JSExceptionBase_h
#define JSExceptionBase_h

namespace JSC {
    
class JSValue;

}   // namespace JSC

namespace WebCore {

class ExceptionBase;

ExceptionBase* toExceptionBase(JSC::JSValue);

} // namespace WebCore

#endif // JSExceptionBase_h
