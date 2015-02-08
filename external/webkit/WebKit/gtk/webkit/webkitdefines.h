

#ifndef webkitdefines_h
#define webkitdefines_h

#include <glib.h>

#ifdef G_OS_WIN32
    #ifdef BUILDING_WEBKIT
        #define WEBKIT_API __declspec(dllexport)
    #else
        #define WEBKIT_API __declspec(dllimport)
    #endif
    #define WEBKIT_OBSOLETE_API WEBKIT_API
#else
    #define WEBKIT_API __attribute__((visibility("default")))
    #define WEBKIT_OBSOLETE_API WEBKIT_API __attribute__((deprecated))
#endif

#ifndef WEBKIT_API
    #define WEBKIT_API
#endif

G_BEGIN_DECLS

typedef struct _WebKitNetworkRequest WebKitNetworkRequest;
typedef struct _WebKitNetworkRequestClass WebKitNetworkRequestClass;

typedef struct _WebKitNetworkResponse WebKitNetworkResponse;
typedef struct _WebKitNetworkResponseClass WebKitNetworkResponseClass;

typedef struct _WebKitWebBackForwardList WebKitWebBackForwardList;
typedef struct _WebKitWebBackForwardListClass WebKitWebBackForwardListClass;

typedef struct _WebKitWebHistoryItem WebKitWebHistoryItem;
typedef struct _WebKitWebHistoryItemClass WebKitWebHistoryItemClass;

typedef struct _WebKitWebFrame WebKitWebFrame;
typedef struct _WebKitWebFrameClass WebKitWebFrameClass;

typedef struct _WebKitWebPolicyDecision WebKitWebPolicyDecision;
typedef struct _WebKitWebPolicyDecisionClass WebKitWebPolicyDecisionClass;

typedef struct _WebKitWebSettings WebKitWebSettings;
typedef struct _WebKitWebSettingsClass WebKitWebSettingsClass;

typedef struct _WebKitWebInspector WebKitWebInspector;
typedef struct _WebKitWebInspectorClass WebKitWebInspectorClass;

typedef struct _WebKitWebWindowFeatures WebKitWebWindowFeatures;
typedef struct _WebKitWebWindowFeaturesClass WebKitWebWindowFeaturesClass;

typedef struct _WebKitWebView WebKitWebView;
typedef struct _WebKitWebViewClass WebKitWebViewClass;

typedef struct _WebKitDownload WebKitDownload;
typedef struct _WebKitDownloadClass WebKitDownloadClass;

typedef struct _WebKitWebResource WebKitWebResource;
typedef struct _WebKitWebResourceClass WebKitWebResourceClass;

typedef struct _WebKitWebDataSource WebKitWebDataSource;
typedef struct _WebKitWebDataSourceClass WebKitWebDataSourceClass;

typedef struct _WebKitWebDatabase WebKitWebDatabase;
typedef struct _WebKitWebDatabaseClass WebKitWebDatabaseClass;

typedef struct _WebKitSecurityOrigin WebKitSecurityOrigin;
typedef struct _WebKitSecurityOriginClass WebKitSecurityOriginClass;

typedef struct _WebKitHitTestResult WebKitHitTestResult;
typedef struct _WebKitHitTestResultClass WebKitHitTestResultClass;

G_END_DECLS

#endif
