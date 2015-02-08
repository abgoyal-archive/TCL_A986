

@class WebGeolocationPositionInternal;

@interface WebGeolocationPosition : NSObject
{
    WebGeolocationPositionInternal *_internal;
}

- (id)initWithTimestamp:(double)timestamp latitude:(double)latitude longitude:(double)longitude accuracy:(double)accuracy;

@end
