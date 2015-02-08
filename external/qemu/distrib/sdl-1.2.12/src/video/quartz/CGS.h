
#include "SDL_config.h"

#define kMaxWindows 256

/* Some of the Core Graphics Server API for obscuring code */
#define kCGSWindowLevelTop          2147483632
#define kCGSWindowLevelDockIconDrag 500
#define kCGSWindowLevelDockMenu     101
#define kCGSWindowLevelMenuIgnore    21
#define kCGSWindowLevelMenu          20
#define kCGSWindowLevelDockLabel     12
#define kCGSWindowLevelDockIcon      11
#define kCGSWindowLevelDock          10
#define kCGSWindowLevelUtility        3
#define kCGSWindowLevelNormal         0


typedef CGError       CGSError;
typedef long          CGSWindowCount;
typedef void *        CGSConnectionID;
typedef int           CGSWindowID;
typedef CGSWindowID*  CGSWindowIDList;
typedef CGWindowLevel CGSWindowLevel;
typedef NSRect        CGSRect;

extern CGSConnectionID _CGSDefaultConnection ();

extern CGSError CGSGetOnScreenWindowList (CGSConnectionID cid,
                                          CGSConnectionID owner,
                                          CGSWindowCount listCapacity,
                                          CGSWindowIDList list,
                                          CGSWindowCount *listCount);

extern CGSError CGSGetScreenRectForWindow (CGSConnectionID cid,
                                           CGSWindowID wid,
                                           CGSRect *rect);

extern CGWindowLevel CGSGetWindowLevel (CGSConnectionID cid,
                                        CGSWindowID wid,
                                        CGSWindowLevel *level);

extern CGSError CGSDisplayHWFill (CGDirectDisplayID id, unsigned int x, unsigned int y,
                                  unsigned int w, unsigned int h, unsigned int color);

extern CGSError CGSDisplayCanHWFill (CGDirectDisplayID id);

extern CGSError CGSGetMouseEnabledFlags (CGSConnectionID cid, CGSWindowID wid, int *flags);

int CGSDisplayHWSync (CGDirectDisplayID id);

