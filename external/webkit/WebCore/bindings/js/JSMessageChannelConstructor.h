

#ifndef JSMessageChannelConstructor_h
#define JSMessageChannelConstructor_h

#include "JSDOMBinding.h"

namespace WebCore {

    class JSMessageChannelConstructor : public DOMConstructorObject {
    public:
        JSMessageChannelConstructor(JSC::ExecState*, JSDOMGlobalObject*);
        virtual ~JSMessageChannelConstructor();
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
        static const JSC::ClassInfo s_info;

        virtual bool implementsHasInstance() const { return true; }
        static JSC::JSObject* construct(JSC::ExecState*, JSC::JSObject*, const JSC::ArgList&);
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
    };

} // namespace WebCore

#endif // JSMessageChannelConstructor_h
