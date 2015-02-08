

#ifndef MediaPlayerPrivateWince_h
#define MediaPlayerPrivateWince_h

#if ENABLE(VIDEO)

#include "MediaPlayerPrivate.h"
#include "Timer.h"
#include <wtf/OwnPtr.h>

namespace WebCore {

    class GraphicsContext;
    class IntSize;
    class IntRect;
    class String;

    class MediaPlayerPrivate : public MediaPlayerPrivateInterface {
    public:
        static void registerMediaEngine(MediaEngineRegistrar);

        ~MediaPlayerPrivate();

        IntSize naturalSize() const;
        bool hasVideo() const;

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

    private:
        MediaPlayerPrivate(MediaPlayer*);

        void updateStates();
        void doSeek();
        void cancelSeek();
        void seekTimerFired(Timer<MediaPlayerPrivate>*);
        float maxTimeLoaded() const;
        void sawUnsupportedTracks();
#if ENABLE(PLUGIN_PROXY_FOR_VIDEO)
        void setMediaPlayerProxy(WebMediaPlayerProxy*);
        void setPoster(const String& url);
        void deliverNotification(MediaPlayerProxyNotificationType);
#endif

        // engine support
        static MediaPlayerPrivateInterface* create(MediaPlayer*);
        static void getSupportedTypes(HashSet<String>& types);
        static MediaPlayer::SupportsType supportsType(const String& type, const String& codecs);
        static bool isAvailable();

        MediaPlayer* m_player;
        float m_seekTo;
        float m_endTime;
        Timer<MediaPlayerPrivate> m_seekTimer;
        MediaPlayer::NetworkState m_networkState;
        MediaPlayer::ReadyState m_readyState;
        unsigned m_enabledTrackCount;
        unsigned m_totalTrackCount;
        bool m_hasUnsupportedTracks;
        bool m_startedPlaying;
        bool m_isStreaming;
#if ENABLE(PLUGIN_PROXY_FOR_VIDEO)
        WebMediaPlayerProxy* m_proxy;
#endif
    };

}

#endif

#endif
