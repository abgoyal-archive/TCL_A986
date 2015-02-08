

#if !defined(ENABLE_DASHBOARD_SUPPORT)
#define ENABLE_DASHBOARD_SUPPORT 1
#endif

#if ENABLE_DASHBOARD_SUPPORT

typedef enum {
    WebDashboardRegionTypeNone,
    WebDashboardRegionTypeCircle,
    WebDashboardRegionTypeRectangle,
    WebDashboardRegionTypeScrollerRectangle
} WebDashboardRegionType;

@interface WebDashboardRegion : NSObject <NSCopying>
{
    NSRect rect;
    NSRect clip;
    WebDashboardRegionType type;
}
- initWithRect:(NSRect)rect clip:(NSRect)clip type:(WebDashboardRegionType)type;
- (NSRect)dashboardRegionClip;
- (NSRect)dashboardRegionRect;
- (WebDashboardRegionType)dashboardRegionType;
@end

#endif
