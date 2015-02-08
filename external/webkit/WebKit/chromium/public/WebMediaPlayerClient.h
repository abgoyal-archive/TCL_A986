

#ifndef WebMediaPlayerClient_h
#define WebMediaPlayerClient_h

namespace WebKit {

class WebRequest;
class WebURL;

class WebMediaPlayerClient {
public:
    virtual void networkStateChanged() = 0;
    virtual void readyStateChanged() = 0;
    virtual void volumeChanged(float) = 0;
    virtual void muteChanged(bool) = 0;
    virtual void timeChanged() = 0;
    virtual void repaint() = 0;
    virtual void durationChanged() = 0;
    virtual void rateChanged() = 0;
    virtual void sizeChanged() = 0;
    virtual void sawUnsupportedTracks() = 0;

protected:
    ~WebMediaPlayerClient() { }
};

} // namespace WebKit

#endif
