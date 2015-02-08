

#ifndef CachedCSSStyleSheet_h
#define CachedCSSStyleSheet_h

#include "CachedResource.h"
#include "TextEncoding.h"
#include <wtf/Vector.h>

namespace WebCore {

    class DocLoader;
    class TextResourceDecoder;

    class CachedCSSStyleSheet : public CachedResource {
    public:
        CachedCSSStyleSheet(const String& URL, const String& charset);
        virtual ~CachedCSSStyleSheet();

        const String sheetText(bool enforceMIMEType = true, bool* hasValidMIMEType = 0) const;

        virtual void didAddClient(CachedResourceClient*);
        
        virtual void allClientsRemoved();

        virtual void setEncoding(const String&);
        virtual String encoding() const;
        virtual void data(PassRefPtr<SharedBuffer> data, bool allDataReceived);
        virtual void error();

        virtual bool schedule() const { return true; }

        void checkNotify();
    
    private:
        bool canUseSheet(bool enforceMIMEType, bool* hasValidMIMEType) const;

    protected:
        RefPtr<TextResourceDecoder> m_decoder;
        String m_decodedSheetText;
    };

}

#endif
