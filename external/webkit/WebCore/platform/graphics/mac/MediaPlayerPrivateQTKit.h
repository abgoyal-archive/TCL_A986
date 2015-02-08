

#ifndef MediaPlayerPrivateQTKit_h
#define MediaPlayerPrivateQTKit_h

#if ENABLE(VIDEO)

#include "MediaPlayerPrivate.h"
#include "Timer.h"
#include "FloatSize.h"
#include <wtf/RetainPtr.h>

#ifdef __OBJC__
#import <QTKit/QTTime.h>
@class QTMovie;
@class QTMovieView;
@class QTMovieLayer;
@class QTVideoRendererWebKitOnly;
@class WebCoreMovieObserver;
#else
class QTMovie;
class QTMovieView;
class QTTime;
class QTMovieLayer;
class QTVideoRendererWebKitOnly;
class WebCoreMovieObserver;
#endif

#ifndef DRAW_FRAME_RATE
#define DRAW_FRAME_RATE 0
#endif

namespace WebCore {

class MediaPlayerPrivate : public MediaPlayerPrivateInterface {
public:
    static void registerMediaEngine(MediaEngineRegistrar);

    ~MediaPlayerPrivate();

    void repaint();
    void loadStateChanged();
    void rateChanged();
    void sizeChanged();
    void timeChanged();
    void didEnd();

private:
    MediaPlayerPrivate(MediaPlayer*);

    // engine support
    static MediaPlayerPrivateInterface* create(MediaPlayer* player);
    static void getSupportedTypes(HashSet<String>& types);
    static MediaPlayer::SupportsType supportsType(const String& type, const String& codecs);
    static bool isAvailable();

    PlatformMedia platformMedia() const;

    IntSize naturalSize() const;
    bool hasVideo() const;
    bool hasAudio() const;
    bool supportsFullscreen() const;
    
    void load(const String& url);
    void cancelLoad();
    
    void play();
    void pause();    
    
    bool paused() const;
    bool seeking() const;
    
    float duration() const;
    float currentTime() const;
    void seek(float time);
    
    void setRate(float);
    void setVolume(float);
    void setPreservesPitch(bool);

    bool hasClosedCaptions() const;
    void setClosedCaptionsVisible(bool);

    MediaPlayer::NetworkState networkState() const { return m_networkState; }
    MediaPlayer::ReadyState readyState() const { return m_readyState; }
    
    PassRefPtr<TimeRanges> buffered() const;
    float maxTimeSeekable() const;
    unsigned bytesLoaded() const;
    unsigned totalBytes() const;
    
    void setVisible(bool);
    void setSize(const IntSize&);
    
    virtual bool hasAvailableVideoFrame() const;

    void paint(GraphicsContext*, const IntRect&);
    void paintCurrentFrameInContext(GraphicsContext*, const IntRect&);

#if USE(ACCELERATED_COMPOSITING)
    bool supportsAcceleratedRendering() const;
    void acceleratedRenderingStateChanged();
#endif

    bool hasSingleSecurityOrigin() const;
    MediaPlayer::MovieLoadType movieLoadType() const;

    void createQTMovie(const String& url);
    void createQTMovie(NSURL *, NSDictionary *movieAttributes);

    enum MediaRenderingMode { MediaRenderingNone, MediaRenderingMovieView, MediaRenderingSoftwareRenderer, MediaRenderingMovieLayer };
    MediaRenderingMode currentRenderingMode() const;
    MediaRenderingMode preferredRenderingMode() const;
    
    void setUpVideoRendering();
    void tearDownVideoRendering();
    bool hasSetUpVideoRendering() const;
    
    void createQTMovieView();
    void detachQTMovieView();
    
    enum QTVideoRendererMode { QTVideoRendererModeDefault, QTVideoRendererModeListensForNewImages };
    void createQTVideoRenderer(QTVideoRendererMode rendererMode);
    void destroyQTVideoRenderer();
    
    void createQTMovieLayer();
    void destroyQTMovieLayer();

    QTTime createQTTime(float time) const;
    
    void updateStates();
    void doSeek();
    void cancelSeek();
    void seekTimerFired(Timer<MediaPlayerPrivate>*);
    float maxTimeLoaded() const;
    void disableUnsupportedTracks();
    
    void sawUnsupportedTracks();
    void cacheMovieScale();
    bool metaDataAvailable() const { return m_qtMovie && m_readyState >= MediaPlayer::HaveMetadata; }

    bool isReadyForRendering() const;
    
    MediaPlayer* m_player;
    RetainPtr<QTMovie> m_qtMovie;
    RetainPtr<QTMovieView> m_qtMovieView;
    RetainPtr<QTVideoRendererWebKitOnly> m_qtVideoRenderer;
    RetainPtr<WebCoreMovieObserver> m_objcObserver;
    float m_seekTo;
    Timer<MediaPlayerPrivate> m_seekTimer;
    MediaPlayer::NetworkState m_networkState;
    MediaPlayer::ReadyState m_readyState;
    IntRect m_rect;
    FloatSize m_scaleFactor;
    unsigned m_enabledTrackCount;
    unsigned m_totalTrackCount;
    float m_reportedDuration;
    float m_cachedDuration;
    float m_timeToRestore;
    RetainPtr<QTMovieLayer> m_qtVideoLayer;
    bool m_startedPlaying;
    bool m_isStreaming;
    bool m_visible;
    bool m_hasUnsupportedTracks;
    bool m_videoFrameHasDrawn;
#if DRAW_FRAME_RATE
    int  m_frameCountWhilePlaying;
    double m_timeStartedPlaying;
    double m_timeStoppedPlaying;
#endif
};

}

#endif
#endif
