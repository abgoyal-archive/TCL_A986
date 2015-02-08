

#ifndef PlatformMenuDescription_h
#define PlatformMenuDescription_h

#if PLATFORM(MAC)
#ifdef __OBJC__
@class NSMutableArray;
#else
class NSMutableArray;
#endif
#elif PLATFORM(WIN)
typedef struct HMENU__* HMENU;
#elif PLATFORM(QT)
#include <qlist.h>
#elif PLATFORM(GTK)
typedef struct _GtkMenu GtkMenu;
#elif PLATFORM(WX)
class wxMenu;
#elif PLATFORM(HAIKU)
class BMenu;
#endif

namespace WebCore {

#if PLATFORM(MAC)
    typedef NSMutableArray* PlatformMenuDescription;
#elif PLATFORM(WIN)
    typedef HMENU PlatformMenuDescription;
#elif PLATFORM(QT)
    class ContextMenuItem;
    typedef const QList<ContextMenuItem>* PlatformMenuDescription;
#elif PLATFORM(GTK)
    typedef GtkMenu* PlatformMenuDescription;
#elif defined ANDROID
// Currently Android is not using this.  Until its use is deemed necessary/desirable, typedef to void*.
    typedef void* PlatformMenuDescription;
#elif PLATFORM(WX)
    typedef wxMenu* PlatformMenuDescription;
#elif PLATFORM(HAIKU)
    typedef BMenu* PlatformMenuDescription;
#else
    typedef void* PlatformMenuDescription;
#endif

} // namespace

#endif // PlatformMenuDescription_h
