

#ifndef JSAudioConstructor_h
#define JSAudioConstructor_h

#if ENABLE(VIDEO)

#include "JSDOMBinding.h"
#include "JSDocument.h"
#include <wtf/RefPtr.h>

namespace WebCore {

    class JSAudioConstructor : public DOMConstructorWithDocument {
    public:
        JSAudioConstructor(JSC::ExecState*, JSDOMGlobalObject*);

        static const JSC::ClassInfo s_info;
    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);

        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

} // namespace WebCore

#endif // ENABLE(VIDEO)

#endif // JSAudioConstructor_h
