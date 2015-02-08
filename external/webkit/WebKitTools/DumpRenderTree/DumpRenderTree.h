

#ifndef DumpRenderTree_h
#define DumpRenderTree_h

// FIXME: Remove this when all platforms are using config.h
#ifndef Config_H
#include <wtf/Platform.h>
#endif

#if PLATFORM(MAC)
#include "DumpRenderTreeMac.h"
#elif PLATFORM(WIN)
#include "DumpRenderTreeWin.h"
#elif PLATFORM(GTK)
#include "DumpRenderTreeGtk.h"
#elif PLATFORM(WX)
#include "DumpRenderTreeWx.h"
#endif

#include <string>

#if !OS(OPENBSD)
std::wstring urlSuitableForTestResult(const std::wstring& url);
#endif

class LayoutTestController;

extern volatile bool done;

// FIXME: This is a bad abstraction.  We should insted pass this to other controller objects which need access to it.
extern LayoutTestController* gLayoutTestController;

void dump();
void displayWebView();

#endif // DumpRenderTree_h
