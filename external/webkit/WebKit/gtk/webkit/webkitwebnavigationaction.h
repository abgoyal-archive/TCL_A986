

#ifndef webkitwebnavigationaction_h
#define webkitwebnavigationaction_h

#include <glib-object.h>

#include <webkit/webkitdefines.h>

G_BEGIN_DECLS

typedef enum {
    WEBKIT_WEB_NAVIGATION_REASON_LINK_CLICKED,
    WEBKIT_WEB_NAVIGATION_REASON_FORM_SUBMITTED,
    WEBKIT_WEB_NAVIGATION_REASON_BACK_FORWARD,
    WEBKIT_WEB_NAVIGATION_REASON_RELOAD,
    WEBKIT_WEB_NAVIGATION_REASON_FORM_RESUBMITTED,
    WEBKIT_WEB_NAVIGATION_REASON_OTHER,
} WebKitWebNavigationReason;

#define WEBKIT_TYPE_WEB_NAVIGATION_ACTION            (webkit_web_navigation_action_get_type())
#define WEBKIT_WEB_NAVIGATION_ACTION(obj)            (G_TYPE_CHECK_INSTANCE_CAST((obj), WEBKIT_TYPE_WEB_NAVIGATION_ACTION, WebKitWebNavigationAction))
#define WEBKIT_WEB_NAVIGATION_ACTION_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST((klass), WEBKIT_TYPE_WEB_NAVIGATION_ACTION, WebKitWebNavigationActionClass))
#define WEBKIT_IS_WEB_NAVIGATION_ACTION(obj)         (G_TYPE_CHECK_INSTANCE_TYPE((obj), WEBKIT_TYPE_WEB_NAVIGATION_ACTION))
#define WEBKIT_IS_WEB_NAVIGATION_ACTION_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), WEBKIT_TYPE_WEB_NAVIGATION_ACTION))
#define WEBKIT_WEB_NAVIGATION_ACTION_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS((obj), WEBKIT_TYPE_WEB_NAVIGATION_ACTION, WebKitWebNavigationActionClass))

typedef struct _WebKitWebNavigationAction WebKitWebNavigationAction;
typedef struct _WebKitWebNavigationActionClass WebKitWebNavigationActionClass;
typedef struct _WebKitWebNavigationActionPrivate WebKitWebNavigationActionPrivate;

struct _WebKitWebNavigationAction {
    GObject parent_instance;

    /*< private >*/
    WebKitWebNavigationActionPrivate* priv;
};

struct _WebKitWebNavigationActionClass {
    GObjectClass parent_class;

    /* Padding for future expansion */
    void (*_webkit_reserved0) (void);
    void (*_webkit_reserved1) (void);
    void (*_webkit_reserved2) (void);
    void (*_webkit_reserved3) (void);
};

WEBKIT_API GType
webkit_web_navigation_action_get_type(void);

WEBKIT_API WebKitWebNavigationReason
webkit_web_navigation_action_get_reason(WebKitWebNavigationAction* navigationAction);

WEBKIT_API void
webkit_web_navigation_action_set_reason(WebKitWebNavigationAction* navigationAction, WebKitWebNavigationReason reason);

WEBKIT_API const gchar*
webkit_web_navigation_action_get_original_uri(WebKitWebNavigationAction* navigationAction);

WEBKIT_API void
webkit_web_navigation_action_set_original_uri(WebKitWebNavigationAction* navigationAction, const gchar* originalUri);

WEBKIT_API gint
webkit_web_navigation_action_get_button(WebKitWebNavigationAction* navigationAction);

WEBKIT_API gint
webkit_web_navigation_action_get_modifier_state(WebKitWebNavigationAction* navigationAction);

WEBKIT_API G_CONST_RETURN gchar *
webkit_web_navigation_action_get_target_frame(WebKitWebNavigationAction* navigationAction);

G_END_DECLS

#endif
