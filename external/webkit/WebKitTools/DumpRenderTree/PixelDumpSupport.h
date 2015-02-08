

#ifndef PixelDumpSupport_h
#define PixelDumpSupport_h

#include <string>

#include <wtf/PassRefPtr.h>

class BitmapContext;

void computeMD5HashStringForBitmapContext(BitmapContext*, char hashString[33]);
PassRefPtr<BitmapContext> createBitmapContextFromWebView(bool onscreen, bool incrementalRepaint, bool sweepHorizontally, bool drawSelectionRect);
void dumpBitmap(BitmapContext*);
void dumpWebViewAsPixelsAndCompareWithExpected(const std::string& expectedHash);
void printPNG(const unsigned char* data, const size_t dataLength);

#if PLATFORM(MAC)

// Can be used as a signal handler
void restoreMainDisplayColorProfile(int ignored);

// May change your color space, requiring a call to restoreMainDisplayColorProfile
void setupMainDisplayColorProfile();

#endif

#endif // PixelDumpSupport_h
