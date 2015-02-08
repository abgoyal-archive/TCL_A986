

#ifndef MediaPlayerProxy_h
#define MediaPlayerProxy_h

#ifdef __OBJC__
@class WebMediaPlayerProxy;
#else
class WebMediaPlayerProxy;
#endif

enum MediaPlayerProxyNotificationType {

    MediaPlayerNotificationMediaValidated = 1,
    MediaPlayerNotificationMediaFailedToValidate,
    
    MediaPlayerNotificationStartUsingNetwork,
    MediaPlayerNotificationStopUsingNetwork,

    MediaPlayerNotificationEnteredFullScreen,
    MediaPlayerNotificationExitedFullScreen,
    
    MediaPlayerNotificationReadyForInspection,
    MediaPlayerNotificationReadyForPlayback,
    MediaPlayerNotificationDidPlayToTheEnd,

    MediaPlayerNotificationPlaybackFailed,

    MediaPlayerNotificationStreamLikelyToKeepUp,
    MediaPlayerNotificationStreamUnlikelyToKeepUp,
    MediaPlayerNotificationStreamBufferFull,
    MediaPlayerNotificationStreamRanDry,
    MediaPlayerNotificationFileLoaded,

    MediaPlayerNotificationSizeDidChange,
    MediaPlayerNotificationVolumeDidChange,
    MediaPlayerNotificationMutedDidChange,
    MediaPlayerNotificationTimeJumped,
    
    MediaPlayerNotificationPlayPauseButtonPressed,
};

#ifdef __OBJC__
@interface NSObject (WebMediaPlayerProxy)

- (int)_interfaceVersion;

- (void)_disconnect;

- (void)_load:(NSURL *)url;
- (void)_cancelLoad;

- (void)_setPoster:(NSURL *)url;

- (void)_play;
- (void)_pause;

- (NSSize)_naturalSize;

- (BOOL)_hasVideo;
- (BOOL)_hasAudio;

- (NSTimeInterval)_duration;

- (double)_currentTime;
- (void)_setCurrentTime:(double)time;
- (BOOL)_seeking;

- (void)_setEndTime:(double)time;

- (float)_rate;
- (void)_setRate:(float)rate;

- (float)_volume;
- (void)_setVolume:(float)newVolume;

- (BOOL)_muted;
- (void)_setMuted:(BOOL)muted;

- (float)_maxTimeBuffered;
- (float)_maxTimeSeekable;
- (NSArray *)_bufferedTimeRanges;

- (int)_dataRate;

- (BOOL)_totalBytesKnown;
- (unsigned)_totalBytes;
- (unsigned)_bytesLoaded;

- (NSArray *)_mimeTypes;

@end
#endif

#endif
