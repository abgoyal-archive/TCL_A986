

#ifndef MediaPlayerPrivateGStreamer_h
#define MediaPlayerPrivateGStreamer_h

#if ENABLE(VIDEO)

#include "MediaPlayerPrivate.h"
#include "Timer.h"

#include <cairo.h>
#include <glib.h>
#include <gst/gst.h>

typedef struct _WebKitVideoSink WebKitVideoSink;
typedef struct _GstBuffer GstBuffer;
typedef struct _GstMessage GstMessage;
typedef struct _GstElement GstElement;
typedef struct _GstBus GstBus;

namespace WebCore {

class GraphicsContext;
class IntSize;
class IntRect;
class String;

gboolean mediaPlayerPrivateMessageCallback(GstBus* bus, GstMessage* message, gpointer data);
void mediaPlayerPrivateVolumeChangedCallback(GObject* element, GParamSpec* pspec, gpointer data);
void mediaPlayerPrivateSourceChangedCallback(GObject* element, GParamSpec* pspec, gpointer data);

class MediaPlayerPrivate : public MediaPlayerPrivateInterface {
        friend gboolean mediaPlayerPrivateMessageCallback(GstBus* bus, GstMessage* message, gpointer data);
        friend void mediaPlayerPrivateRepaintCallback(WebKitVideoSink*, GstBuffer* buffer, MediaPlayerPrivate* playerPrivate);
        friend void mediaPlayerPrivateSourceChangedCallback(GObject* element, GParamSpec* pspec, gpointer data);

        public:
            static void registerMediaEngine(MediaEngineRegistrar);
            ~MediaPlayerPrivate();

            IntSize naturalSize() const;
            bool hasVideo() const;
            bool hasAudio() const;

            void load(const String &url);
            void cancelLoad();
            bool loadNextLocation();

            void play();
            void pause();

            bool paused() const;
            bool seeking() const;

            float duration() const;
            float currentTime() const;
            void seek(float);

            void setRate(float);

            void setVolume(float);
            void volumeChanged();
            void volumeChangedCallback();

            bool supportsMuting() const;
            void setMuted(bool);
            void muteChanged();
            void muteChangedCallback();

            MediaPlayer::NetworkState networkState() const;
            MediaPlayer::ReadyState readyState() const;

            PassRefPtr<TimeRanges> buffered() const;
            float maxTimeSeekable() const;
            unsigned bytesLoaded() const;
            unsigned totalBytes() const;

            void setVisible(bool);
            void setSize(const IntSize&);

            void mediaLocationChanged(GstMessage*);
            void loadStateChanged();
            void sizeChanged();
            void timeChanged();
            void didEnd();
            void durationChanged();
            void loadingFailed(MediaPlayer::NetworkState);

            void repaint();
            void paint(GraphicsContext*, const IntRect&);

            bool hasSingleSecurityOrigin() const;

            bool supportsFullscreen() const;

            bool pipelineReset() const { return m_resetPipeline; }

        private:
            MediaPlayerPrivate(MediaPlayer*);
            static MediaPlayerPrivateInterface* create(MediaPlayer* player);

            static void getSupportedTypes(HashSet<String>&);
            static MediaPlayer::SupportsType supportsType(const String& type, const String& codecs);
            static bool isAvailable();

            void updateStates();
            void cancelSeek();
            void endPointTimerFired(Timer<MediaPlayerPrivate>*);
            float maxTimeLoaded() const;
            void startEndPointTimerIfNeeded();

            void createGSTPlayBin(String url);
            bool changePipelineState(GstState state);

        private:
            MediaPlayer* m_player;
            GstElement* m_playBin;
            GstElement* m_videoSink;
            GstElement* m_fpsSink;
            GstElement* m_source;
            GstClockTime m_seekTime;
            bool m_changingRate;
            float m_endTime;
            bool m_isEndReached;
            MediaPlayer::NetworkState m_networkState;
            MediaPlayer::ReadyState m_readyState;
            bool m_startedPlaying;
            mutable bool m_isStreaming;
            IntSize m_size;
            GstBuffer* m_buffer;
            GstStructure* m_mediaLocations;
            gint m_mediaLocationCurrentIndex;
            bool m_resetPipeline;
            bool m_paused;
            bool m_seeking;
            float m_playbackRate;
            bool m_errorOccured;
            guint m_volumeIdleId;
            gfloat m_mediaDuration;
            guint m_muteIdleId;
    };
}

#endif
#endif
