

#ifndef ScriptSourceProvider_h
#define ScriptSourceProvider_h

#include <parser/SourceProvider.h>

namespace WebCore {

    class String;

    class ScriptSourceProvider : public JSC::SourceProvider {
    public:
        ScriptSourceProvider(const JSC::UString& url, JSC::SourceBOMPresence hasBOMs = JSC::SourceCouldHaveBOMs)
            : SourceProvider(url, hasBOMs)
        {
        }

        virtual const String& source() const = 0;
    };

} // namespace WebCore

#endif // ScriptSourceProvider_h
