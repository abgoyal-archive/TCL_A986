

#ifndef Icon_h
#define Icon_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Forward.h>
#include <wtf/Vector.h>

#if PLATFORM(MAC)
#include <wtf/RetainPtr.h>
#ifdef __OBJC__
@class NSImage;
#else
class NSImage;
#endif
#elif PLATFORM(WIN)
typedef struct HICON__* HICON;
#elif PLATFORM(QT)
#include <QIcon>
#elif PLATFORM(GTK)
typedef struct _GdkPixbuf GdkPixbuf;
#elif PLATFORM(CHROMIUM)
#include "PlatformIcon.h"
#endif

namespace WebCore {

class GraphicsContext;
class IntRect;
class String;
    
class Icon : public RefCounted<Icon> {
public:
    static PassRefPtr<Icon> createIconForFiles(const Vector<String>& filenames);

    ~Icon();

    void paint(GraphicsContext*, const IntRect&);

#if PLATFORM(WIN)
    static PassRefPtr<Icon> create(HICON hIcon) { return adoptRef(new Icon(hIcon)); }
#endif

private:
#if PLATFORM(MAC)
    Icon(NSImage*);
    RetainPtr<NSImage> m_nsImage;
#elif PLATFORM(WIN)
    Icon(HICON);
    HICON m_hIcon;
#elif PLATFORM(QT)
    Icon();
    QIcon m_icon;
#elif PLATFORM(GTK)
    Icon();
    GdkPixbuf* m_icon;
#elif PLATFORM(CHROMIUM)
    Icon(const PlatformIcon&);
    PlatformIcon m_icon;
#endif
};

}

#endif
