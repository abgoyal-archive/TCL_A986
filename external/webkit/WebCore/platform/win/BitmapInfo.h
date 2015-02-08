

#ifndef BitmapInfo_h
#define BitmapInfo_h

#include <windows.h>
#include "IntSize.h"

namespace WebCore {

struct BitmapInfo : public BITMAPINFO {
    BitmapInfo ();
    static BitmapInfo create(const IntSize&);
    static BitmapInfo createBottomUp(const IntSize&);
};

} // namespace WebCore

#endif // BitmapInfo_h
