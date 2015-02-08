

#ifndef MediaDocument_h
#define MediaDocument_h

#if ENABLE(VIDEO)

#include "HTMLDocument.h"

namespace WebCore {

class MediaDocument : public HTMLDocument {
public:
    static PassRefPtr<MediaDocument> create(Frame* frame)
    {
        return adoptRef(new MediaDocument(frame));
    }
    virtual ~MediaDocument();

    void mediaElementSawUnsupportedTracks();

private:
    MediaDocument(Frame*);

    virtual bool isMediaDocument() const { return true; }        
    virtual Tokenizer* createTokenizer();

    virtual void defaultEventHandler(Event*);

    void replaceMediaElementTimerFired(Timer<MediaDocument>*);

    Timer<MediaDocument> m_replaceMediaElementTimer;
};
    
}

#endif
#endif
