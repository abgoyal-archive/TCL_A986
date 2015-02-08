

#ifndef MediaPlayerPrivatePhonon_h
#define MediaPlayerPrivatePhonon_h

#include "MediaPlayerPrivate.h"

#include <QObject>
#include <phononnamespace.h>

QT_BEGIN_NAMESPACE
class QWidget;
class QUrl;

namespace Phonon {
    class MediaObject;
    class VideoWidget;
    class AudioOutput;
    class MediaSource;
}
QT_END_NAMESPACE

namespace WebCore {

    class MediaPlayerPrivate : public QObject, public MediaPlayerPrivateInterface {

        Q_OBJECT

    public:
        static void registerMediaEngine(MediaEngineRegistrar);
        ~MediaPlayerPrivate();

        // These enums are used for debugging
        Q_ENUMS(ReadyState NetworkState PhononState)

        enum ReadyState {
            HaveNothing, 
            HaveMetadata, 
            HaveCurrentData, 
            HaveFutureData, 
            HaveEnoughData
        };

        enum NetworkState {
            Empty, 
            Idle, 
            Loading, 
            Loaded, 
            FormatError, 
            NetworkError, 
            DecodeError
        };

        enum PhononState {
            LoadingState,
            StoppedState,
            PlayingState,
            BufferingState,
            PausedState,
            ErrorState
        };

        IntSize naturalSize() const;
        bool hasVideo() const;
        bool hasAudio() const;

        void load(const String &url);
        void cancelLoad();

        void play();
        void pause();

        bool paused() const;
        bool seeking() const;

        float duration() const;
        float currentTime() const;
        void seek(float);

        void setRate(float);
        void setVolume(float);
        void setMuted(bool);

        MediaPlayer::NetworkState networkState() const;
        MediaPlayer::ReadyState readyState() const;

        PassRefPtr<TimeRanges> buffered() const;
        float maxTimeSeekable() const;
        unsigned bytesLoaded() const;
        unsigned totalBytes() const;

        void setVisible(bool);
        void setSize(const IntSize&);

        void paint(GraphicsContext*, const IntRect&);

    protected:
        bool eventFilter(QObject*, QEvent*);

    private slots:
        void stateChanged(Phonon::State, Phonon::State);
        void metaDataChanged();
        void seekableChanged(bool);
        void hasVideoChanged(bool);
        void bufferStatus(int);
        void finished();
        void currentSourceChanged(const Phonon::MediaSource&);
        void aboutToFinish();
        void totalTimeChanged(qint64);

    private:
        MediaPlayerPrivate(MediaPlayer*);
        static MediaPlayerPrivateInterface* create(MediaPlayer* player);

        static void getSupportedTypes(HashSet<String>&);
        static MediaPlayer::SupportsType supportsType(const String& type, const String& codecs);
        static bool isAvailable() { return true; }

        void updateStates();

        MediaPlayer* m_player;

        MediaPlayer::NetworkState m_networkState;
        MediaPlayer::ReadyState m_readyState;

        Phonon::MediaObject* m_mediaObject;
        Phonon::VideoWidget* m_videoWidget;
        Phonon::AudioOutput* m_audioOutput;

        bool m_isVisible;
    };
}

#endif // MediaPlayerPrivatePhonon_h
