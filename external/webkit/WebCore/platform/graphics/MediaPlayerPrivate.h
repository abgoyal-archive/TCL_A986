

#ifndef MediaPlayerPrivate_h
#define MediaPlayerPrivate_h

#if ENABLE(VIDEO)

#include "MediaPlayer.h"

namespace WebCore {

class IntRect;
class IntSize;
class String;

class MediaPlayerPrivateInterface : public Noncopyable {
public:
    virtual ~MediaPlayerPrivateInterface() { }

    virtual void load(const String& url) = 0;
    virtual void cancelLoad() = 0;
    
    virtual void prepareToPlay() { }
    virtual PlatformMedia platformMedia() const { return NoPlatformMedia; }

    virtual void play() = 0;
    virtual void pause() = 0;    

    virtual bool supportsFullscreen() const { return false; }
    virtual bool supportsSave() const { return false; }

    virtual IntSize naturalSize() const = 0;

    virtual bool hasVideo() const = 0;
    virtual bool hasAudio() const = 0;

    virtual void setVisible(bool) = 0;

    virtual float duration() const = 0;

    virtual float currentTime() const = 0;
    virtual void seek(float time) = 0;
    virtual bool seeking() const = 0;

    virtual float startTime() const { return 0; }

    virtual void setRate(float) = 0;
    virtual void setPreservesPitch(bool) { }

    virtual bool paused() const = 0;

    virtual void setVolume(float) = 0;

    virtual bool supportsMuting() const { return false; }
    virtual void setMuted(bool) { }

    virtual bool hasClosedCaptions() const { return false; }    
    virtual void setClosedCaptionsVisible(bool) { }

    virtual MediaPlayer::NetworkState networkState() const = 0;
    virtual MediaPlayer::ReadyState readyState() const = 0;

    virtual float maxTimeSeekable() const = 0;
    virtual PassRefPtr<TimeRanges> buffered() const = 0;

    virtual unsigned bytesLoaded() const = 0;

    virtual void setSize(const IntSize&) = 0;

    virtual void paint(GraphicsContext*, const IntRect&) = 0;

    virtual void paintCurrentFrameInContext(GraphicsContext* c, const IntRect& r) { paint(c, r); }

    virtual void setAutobuffer(bool) { };

    virtual bool hasAvailableVideoFrame() const { return readyState() >= MediaPlayer::HaveCurrentData; }

    virtual bool canLoadPoster() const { return false; }
    virtual void setPoster(const String&) { }

#if ENABLE(PLUGIN_PROXY_FOR_VIDEO)
    virtual void deliverNotification(MediaPlayerProxyNotificationType) = 0;
    virtual void setMediaPlayerProxy(WebMediaPlayerProxy*) = 0;
#endif

#if USE(ACCELERATED_COMPOSITING)
    // whether accelerated rendering is supported by the media engine for the current media.
    virtual bool supportsAcceleratedRendering() const { return false; }
    // called when the rendering system flips the into or out of accelerated rendering mode.
    virtual void acceleratedRenderingStateChanged() { }
#endif

    virtual bool hasSingleSecurityOrigin() const { return false; }

    virtual MediaPlayer::MovieLoadType movieLoadType() const { return MediaPlayer::Unknown; }

};

}

#endif
#endif
