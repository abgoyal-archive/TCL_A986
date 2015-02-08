

#ifndef CachedXBLDocument_h
#define CachedXBLDocument_h

#include "CachedResource.h"
#include <wtf/Vector.h>

namespace WebCore {
    class CachedResource;
    class Request;
    class DocLoader;
    class TextResourceDecoder;
    class CachedResourceClient;
    
#if ENABLE(XBL)
    class CachedXBLDocument : public CachedResource {
    public:
        CachedXBLDocument(const String& url);
        virtual ~CachedXBLDocument();
        
        XBL::XBLDocument* document() const { return m_document; }
        
        virtual void addClient(CachedResourceClient*);
        
        virtual void setEncoding(const String&);
        virtual String encoding() const;
        virtual void data(Vector<char>&, bool allDataReceived);
        virtual void error();
        
        virtual bool schedule() const { return true; }
        
        void checkNotify();
        
    protected:
        XBL::XBLDocument* m_document;
        RefPtr<TextResourceDecoder> m_decoder;
    };

#endif

}

#endif
