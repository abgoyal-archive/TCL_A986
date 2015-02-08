

#ifndef WebMediaPlayerClientImpl_h
#define WebMediaPlayerClientImpl_h

#if ENABLE(VIDEO)

#include "MediaPlayerPrivate.h"
#include "WebMediaPlayerClient.h"
#include <wtf/OwnPtr.h>

namespace WebKit {

class WebMediaPlayer;

// This class serves as a bridge between WebCore::MediaPlayer and
// WebKit::WebMediaPlayer.
class WebMediaPlayerClientImpl : public WebMediaPlayerClient
                               , public WebCore::MediaPlayerPrivateInterface {
public:
    static bool isEnabled();
    static void setIsEnabled(bool);
    static void registerSelf(WebCore::MediaEngineRegistrar);

    // WebMediaPlayerClient methods:
    virtual void networkStateChanged();
    virtual void readyStateChanged();
    virtual void volumeChanged(float);
    virtual void muteChanged(bool);
    virtual void timeChanged();
    virtual void repaint();
    virtual void durationChanged();
    virtual void rateChanged();
    virtual void sizeChanged();
    virtual void sawUnsupportedTracks();

    // MediaPlayerPrivateInterface methods:
    virtual void load(const WebCore::String& url);
    virtual void cancelLoad();
    virtual void play();
    virtual void pause();
    virtual bool supportsFullscreen() const;
    virtual bool supportsSave() const;
    virtual WebCore::IntSize naturalSize() const;
    virtual bool hasVideo() const;
    virtual bool hasAudio() const;
    virtual void setVisible(bool);
    virtual float duration() const;
    virtual float currentTime() const;
    virtual void seek(float time);
    virtual bool seeking() const;
    virtual void setEndTime(float time);
    virtual void setRate(float);
    virtual bool paused() const;
    virtual void setVolume(float);
    virtual WebCore::MediaPlayer::NetworkState networkState() const;
    virtual WebCore::MediaPlayer::ReadyState readyState() const;
    virtual float maxTimeSeekable() const;
    virtual WTF::PassRefPtr<WebCore::TimeRanges> buffered() const;
    virtual int dataRate() const;
    virtual void setAutobuffer(bool);
    virtual bool totalBytesKnown() const;
    virtual unsigned totalBytes() const;
    virtual unsigned bytesLoaded() const;
    virtual void setSize(const WebCore::IntSize&);
    virtual void paint(WebCore::GraphicsContext*, const WebCore::IntRect&);
    virtual bool hasSingleSecurityOrigin() const;
    virtual WebCore::MediaPlayer::MovieLoadType movieLoadType() const;

private:
    WebMediaPlayerClientImpl();

    static WebCore::MediaPlayerPrivateInterface* create(WebCore::MediaPlayer*);
    static void getSupportedTypes(WTF::HashSet<WebCore::String>&);
    static WebCore::MediaPlayer::SupportsType supportsType(
        const WebCore::String& type, const WebCore::String& codecs);

    WebCore::MediaPlayer* m_mediaPlayer;
    OwnPtr<WebMediaPlayer> m_webMediaPlayer;
    static bool m_isEnabled;
};

} // namespace WebKit

#endif

#endif
