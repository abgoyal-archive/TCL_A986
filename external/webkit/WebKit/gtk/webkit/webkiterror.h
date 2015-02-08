

#ifndef webkiterror_h
#define webkiterror_h

#include <glib.h>

#include <webkit/webkitdefines.h>

G_BEGIN_DECLS

#define WEBKIT_NETWORK_ERROR        webkit_network_error_quark ()
#define WEBKIT_POLICY_ERROR         webkit_policy_error_quark  ()
#define WEBKIT_PLUGIN_ERROR         webkit_plugin_error_quark  ()

typedef enum {
    WEBKIT_NETWORK_ERROR_FAILED                                 = 399,
    WEBKIT_NETWORK_ERROR_TRANSPORT                              = 300,
    WEBKIT_NETWORK_ERROR_UNKNOWN_PROTOCOL                       = 301,
    WEBKIT_NETWORK_ERROR_CANCELLED                              = 302,
    WEBKIT_NETWORK_ERROR_FILE_DOES_NOT_EXIST                    = 303,
} WebKitNetworkError;

/* Sync'd with Mac's WebKit Errors */
typedef enum {
    WEBKIT_POLICY_ERROR_FAILED                                  = 199,
    WEBKIT_POLICY_ERROR_CANNOT_SHOW_MIME_TYPE                   = 100,
    WEBKIT_POLICY_ERROR_CANNOT_SHOW_URL                         = 101,
    WEBKIT_POLICY_ERROR_FRAME_LOAD_INTERRUPTED_BY_POLICY_CHANGE = 102,
    WEBKIT_POLICY_ERROR_CANNOT_USE_RESTRICTED_PORT              = 103,
} WebKitPolicyError;

typedef enum {
    WEBKIT_PLUGIN_ERROR_FAILED                                  = 299,
    WEBKIT_PLUGIN_ERROR_CANNOT_FIND_PLUGIN                      = 200,
    WEBKIT_PLUGIN_ERROR_CANNOT_LOAD_PLUGIN                      = 201,
    WEBKIT_PLUGIN_ERROR_JAVA_UNAVAILABLE                        = 202,
    WEBKIT_PLUGIN_ERROR_CONNECTION_CANCELLED                    = 203,
    WEBKIT_PLUGIN_ERROR_WILL_HANDLE_LOAD                        = 204,
} WebKitPluginError;


WEBKIT_API GQuark
webkit_network_error_quark (void);

WEBKIT_API GQuark
webkit_policy_error_quark  (void);

WEBKIT_API GQuark
webkit_plugin_error_quark  (void);

G_END_DECLS

#endif
