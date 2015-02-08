

#ifndef webkitwebpolicydecision_h
#define webkitwebpolicydecision_h

#include <glib-object.h>
#include <stdint.h>
#include "webkitdefines.h"

G_BEGIN_DECLS

#define WEBKIT_TYPE_WEB_POLICY_DECISION            (webkit_web_policy_decision_get_type())
#define WEBKIT_WEB_POLICY_DECISION(obj)            (G_TYPE_CHECK_INSTANCE_CAST((obj), WEBKIT_TYPE_WEB_POLICY_DECISION, WebKitWebPolicyDecision))
#define WEBKIT_WEB_POLICY_DECISION_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST((klass),  WEBKIT_TYPE_WEB_POLICY_DECISION, WebKitWebPolicyDecisionClass))
#define WEBKIT_IS_WEB_POLICY_DECISION(obj)         (G_TYPE_CHECK_INSTANCE_TYPE((obj), WEBKIT_TYPE_WEB_POLICY_DECISION))
#define WEBKIT_IS_WEB_POLICY_DECISION_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass),  WEBKIT_TYPE_WEB_POLICY_DECISION))
#define WEBKIT_WEB_POLICY_DECISION_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS((obj),  WEBKIT_TYPE_WEB_POLICY_DECISION, WebKitWebPolicyDecisionClass))

typedef struct _WebKitWebPolicyDecisionPrivate WebKitWebPolicyDecisionPrivate;
struct _WebKitWebPolicyDecision {
    GObject parent_instance;

    /*< private >*/
    WebKitWebPolicyDecisionPrivate* priv;
};

struct _WebKitWebPolicyDecisionClass {
    GObjectClass parent_class;

    /* Padding for future expansion */
    void (*_webkit_reserved0) (void);
    void (*_webkit_reserved1) (void);
    void (*_webkit_reserved2) (void);
    void (*_webkit_reserved3) (void);
};

WEBKIT_API GType
webkit_web_policy_decision_get_type (void);

WEBKIT_API void
webkit_web_policy_decision_use (WebKitWebPolicyDecision* decision);

WEBKIT_API void
webkit_web_policy_decision_ignore (WebKitWebPolicyDecision* decision);

WEBKIT_API void
webkit_web_policy_decision_download (WebKitWebPolicyDecision* decision);

G_END_DECLS

#endif
