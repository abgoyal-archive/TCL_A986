

#ifndef CachedXSLStyleSheet_h
#define CachedXSLStyleSheet_h

#include "CachedResource.h"
#include <wtf/Vector.h>

namespace WebCore {

    class DocLoader;
    class TextResourceDecoder;

#if ENABLE(XSLT)
    class CachedXSLStyleSheet : public CachedResource {
    public:
        CachedXSLStyleSheet(const String& url);

        const String& sheet() const { return m_sheet; }
        
        virtual void didAddClient(CachedResourceClient*);
        
        virtual void setEncoding(const String&);
        virtual String encoding() const;
        virtual void data(PassRefPtr<SharedBuffer> data, bool allDataReceived);
        virtual void error();
        
        virtual bool schedule() const { return true; }
        
        void checkNotify();
        
    protected:
        String m_sheet;
        RefPtr<TextResourceDecoder> m_decoder;
    };

#endif

}

#endif
