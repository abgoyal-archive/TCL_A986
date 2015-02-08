

#ifndef WebMediaPlayer_h
#define WebMediaPlayer_h

#include "WebCanvas.h"
#include "WebVector.h"

namespace WebKit {

class WebMediaPlayerClient;
class WebURL;
struct WebRect;
struct WebSize;

struct WebTimeRange {
    WebTimeRange() : start(0), end(0) {}
    WebTimeRange(float s, float e) : start(s), end(e) {}

    float start;
    float end;
};

typedef WebVector<WebTimeRange> WebTimeRanges;

class WebMediaPlayer {
public:
    enum NetworkState {
        Empty,
        Idle,
        Loading,
        Loaded,
        FormatError,
        NetworkError,
        DecodeError,
    };

    enum ReadyState {
        HaveNothing,
        HaveMetadata,
        HaveCurrentData,
        HaveFutureData,
        HaveEnoughData,
    };

    enum MovieLoadType {
        Unknown,
        Download,
        StoredStream,
        LiveStream,
    };

    virtual ~WebMediaPlayer() {}

    virtual void load(const WebURL&) = 0;
    virtual void cancelLoad() = 0;

    // Playback controls.
    virtual void play() = 0;
    virtual void pause() = 0;
    virtual bool supportsFullscreen() const = 0;
    virtual bool supportsSave() const = 0;
    virtual void seek(float seconds) = 0;
    virtual void setEndTime(float seconds) = 0;
    virtual void setRate(float) = 0;
    virtual void setVolume(float) = 0;
    virtual void setVisible(bool) = 0;
    virtual bool setAutoBuffer(bool) = 0;
    virtual bool totalBytesKnown() = 0;
    virtual const WebTimeRanges& buffered() const = 0;
    virtual float maxTimeSeekable() const = 0;

    virtual void setSize(const WebSize&) = 0;

    virtual void paint(WebCanvas*, const WebRect&) = 0;

    // True if the loaded media has a playable video/audio track.
    virtual bool hasVideo() const = 0;
    virtual bool hasAudio() const = 0;

    // Dimension of the video.
    virtual WebSize naturalSize() const = 0;

    // Getters of playback state.
    virtual bool paused() const = 0;
    virtual bool seeking() const = 0;
    virtual float duration() const = 0;
    virtual float currentTime() const = 0;

    // Get rate of loading the resource.
    virtual int dataRate() const = 0;

    // Internal states of loading and network.
    virtual NetworkState networkState() const = 0;
    virtual ReadyState readyState() const = 0;

    virtual unsigned long long bytesLoaded() const = 0;
    virtual unsigned long long totalBytes() const = 0;

    virtual bool hasSingleSecurityOrigin() const = 0;
    virtual MovieLoadType movieLoadType() const = 0;
};

} // namespace WebKit

#endif
