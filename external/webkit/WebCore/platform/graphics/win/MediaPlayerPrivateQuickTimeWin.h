

#ifndef MediaPlayerPrivateQTKit_h
#define MediaPlayerPrivateQTKit_h

#if ENABLE(VIDEO)

#include "MediaPlayerPrivate.h"
#include "Timer.h"
#include <QTMovieWin.h>
#include <wtf/OwnPtr.h>
#include <wtf/RetainPtr.h>

#if USE(ACCELERATED_COMPOSITING)
#include "GraphicsLayerClient.h"
#endif 

#ifndef DRAW_FRAME_RATE
#define DRAW_FRAME_RATE 0
#endif

typedef struct CGImage *CGImageRef;

namespace WebCore {

class GraphicsContext;
class IntSize;
class IntRect;
class String;

class MediaPlayerPrivate : public MediaPlayerPrivateInterface, public QTMovieWinClient 
#if USE(ACCELERATED_COMPOSITING)
        , public GraphicsLayerClient
#endif 
{
public:
    static void registerMediaEngine(MediaEngineRegistrar);

    ~MediaPlayerPrivate();

private:

#if USE(ACCELERATED_COMPOSITING)
    // GraphicsLayerClient methods
    virtual void paintContents(const GraphicsLayer*, GraphicsContext&, GraphicsLayerPaintingPhase, const IntRect& inClip);
    virtual void notifyAnimationStarted(const GraphicsLayer*, double time) { }
    virtual void notifySyncRequired(const GraphicsLayer*);
    virtual bool showDebugBorders() const { return false; }
    virtual bool showRepaintCounter() const { return false; }
#endif 

    MediaPlayerPrivate(MediaPlayer*);

    virtual bool supportsFullscreen() const;
    virtual PlatformMedia platformMedia() const;

    IntSize naturalSize() const;
    bool hasVideo() const;
    bool hasAudio() const;

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
    
    MediaPlayer::NetworkState networkState() const { return m_networkState; }
    MediaPlayer::ReadyState readyState() const { return m_readyState; }
    
    PassRefPtr<TimeRanges> buffered() const;
    float maxTimeSeekable() const;
    unsigned bytesLoaded() const;
    unsigned totalBytes() const;
    
    void setVisible(bool);
    void setSize(const IntSize&);
    
    void loadStateChanged();
    void didEnd();
    
    void paint(GraphicsContext*, const IntRect&);
    void paintCompleted(GraphicsContext&, const IntRect&);

    bool hasSingleSecurityOrigin() const;

    bool hasClosedCaptions() const;
    void setClosedCaptionsVisible(bool);

    void updateStates();
    void doSeek();
    void cancelSeek();
    void seekTimerFired(Timer<MediaPlayerPrivate>*);
    float maxTimeLoaded() const;
    void sawUnsupportedTracks();

    virtual void movieEnded(QTMovieWin*);
    virtual void movieLoadStateChanged(QTMovieWin*);
    virtual void movieTimeChanged(QTMovieWin*);
    virtual void movieNewImageAvailable(QTMovieWin*);

    // engine support
    static MediaPlayerPrivateInterface* create(MediaPlayer*);
    static void getSupportedTypes(HashSet<String>& types);
    static MediaPlayer::SupportsType supportsType(const String& type, const String& codecs);
    static bool isAvailable();

#if USE(ACCELERATED_COMPOSITING)
    virtual bool supportsAcceleratedRendering() const;
    virtual void acceleratedRenderingStateChanged();
#endif

    enum MediaRenderingMode { MediaRenderingNone, MediaRenderingSoftwareRenderer, MediaRenderingMovieLayer };
    MediaRenderingMode currentRenderingMode() const;
    MediaRenderingMode preferredRenderingMode() const;
    bool isReadyForRendering() const;

    void setUpVideoRendering();
    void tearDownVideoRendering();
    bool hasSetUpVideoRendering() const;

    void createLayerForMovie();
    void destroyLayerForMovie();

    MediaPlayer* m_player;
    OwnPtr<QTMovieWin> m_qtMovie;
#if USE(ACCELERATED_COMPOSITING)
    OwnPtr<GraphicsLayer> m_qtVideoLayer;
#endif
    float m_seekTo;
    Timer<MediaPlayerPrivate> m_seekTimer;
    IntSize m_size;
    MediaPlayer::NetworkState m_networkState;
    MediaPlayer::ReadyState m_readyState;
    unsigned m_enabledTrackCount;
    unsigned m_totalTrackCount;
    bool m_hasUnsupportedTracks;
    bool m_startedPlaying;
    bool m_isStreaming;
    bool m_visible;
    bool m_newFrameAvailable;
#if DRAW_FRAME_RATE
    double m_frameCountWhilePlaying;
    double m_timeStartedPlaying;
    double m_timeStoppedPlaying;
#endif
};

}

#endif
#endif
