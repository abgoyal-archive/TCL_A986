

#ifndef SkBitmapRef_DEFINED
#define SkBitmapRef_DEFINED 

#include "SkRefCnt.h"
#include "SkBitmap.h"

class SkBitmapRef : public SkRefCnt {
public:
    SkBitmapRef() : fOrigWidth(0), fOrigHeight(0), fAccessed(false) {}
    explicit SkBitmapRef(const SkBitmap& src)
        : fBitmap(src),
          fOrigWidth(src.width()),
          fOrigHeight(src.height()),
          fAccessed(false) {}

    const SkBitmap& bitmap() const { return fBitmap; }
    SkBitmap& bitmap() { return fBitmap; }

    int origWidth() const { return fOrigWidth; }
    int origHeight() const { return fOrigHeight; }
    
    void setOrigSize(int width, int height) {
        fOrigWidth = width;
        fOrigHeight = height;
    }
    // return true if this is not the first access
    // mark it true so all subsequent calls return true
    bool accessed() { bool result = fAccessed; 
        fAccessed = true; return result; }

private:
    SkBitmap fBitmap;
    int      fOrigWidth, fOrigHeight;
    bool     fAccessed;
};

#endif
