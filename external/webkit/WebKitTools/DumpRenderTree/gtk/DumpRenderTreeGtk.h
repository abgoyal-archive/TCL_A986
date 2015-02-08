

#ifndef DumpRenderTreeGtk_h
#define DumpRenderTreeGtk_h

#include <webkit/webkitdefines.h>
#include <JavaScriptCore/JSBase.h>

#include <glib.h>

extern WebKitWebFrame* mainFrame;
extern WebKitWebFrame* topLoadingFrame;
extern guint waitToDumpWatchdog;
extern bool waitForPolicy;
extern GSList* webViewList;

gchar* JSStringCopyUTF8CString(JSStringRef jsString);

#endif // DumpRenderTreeGtk_h
