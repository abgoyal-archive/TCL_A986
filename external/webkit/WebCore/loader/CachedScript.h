

#ifndef CachedScript_h
#define CachedScript_h

#include "CachedResource.h"
#include "Timer.h"

namespace WebCore {

    class DocLoader;
    class TextResourceDecoder;

    class CachedScript : public CachedResource {
    public:
        CachedScript(const String& url, const String& charset);
        virtual ~CachedScript();

        const String& script();

        virtual void didAddClient(CachedResourceClient*);
        virtual void allClientsRemoved();

        virtual void setEncoding(const String&);
        virtual String encoding() const;
        virtual void data(PassRefPtr<SharedBuffer> data, bool allDataReceived);
        virtual void error();

        virtual bool schedule() const { return false; }

        void checkNotify();

        virtual void destroyDecodedData();

    private:
        void decodedDataDeletionTimerFired(Timer<CachedScript>*);

        String m_script;
        RefPtr<TextResourceDecoder> m_decoder;
        Timer<CachedScript> m_decodedDataDeletionTimer;
    };
}

#endif
