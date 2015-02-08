

#include "config.h"

#include <wtf/Assertions.h>
#include "FrameLoaderTypes.h"

#include <glib/gi18n-lib.h>
#include "webkitwebnavigationaction.h"
#include "webkitprivate.h"
#include "webkitenumtypes.h"

#include <string.h>

static void webkit_web_navigation_action_set_target_frame(WebKitWebNavigationAction* navigationAction, const gchar* targetFrame);


struct _WebKitWebNavigationActionPrivate {
    WebKitWebNavigationReason reason;
    gchar* originalUri;
    gint button;
    gint modifier_state;
    gchar* targetFrame;
};

#define WEBKIT_WEB_NAVIGATION_ACTION_GET_PRIVATE(obj)(G_TYPE_INSTANCE_GET_PRIVATE((obj), WEBKIT_TYPE_WEB_NAVIGATION_ACTION, WebKitWebNavigationActionPrivate))

enum  {
    PROP_0,

    PROP_REASON,
    PROP_ORIGINAL_URI,
    PROP_BUTTON,
    PROP_MODIFIER_STATE,
    PROP_TARGET_FRAME
};

G_DEFINE_TYPE(WebKitWebNavigationAction, webkit_web_navigation_action, G_TYPE_OBJECT)


static void webkit_web_navigation_action_get_property(GObject* object, guint propertyId, GValue* value, GParamSpec* pspec)
{
    WebKitWebNavigationAction* navigationAction = WEBKIT_WEB_NAVIGATION_ACTION(object);

    switch(propertyId) {
    case PROP_REASON:
        g_value_set_enum(value, webkit_web_navigation_action_get_reason(navigationAction));
        break;
    case PROP_ORIGINAL_URI:
        g_value_set_string(value, webkit_web_navigation_action_get_original_uri(navigationAction));
        break;
    case PROP_BUTTON:
        g_value_set_int(value, webkit_web_navigation_action_get_button(navigationAction));
        break;
    case PROP_MODIFIER_STATE:
        g_value_set_int(value, webkit_web_navigation_action_get_modifier_state(navigationAction));
        break;
    case PROP_TARGET_FRAME:
        g_value_set_string(value, webkit_web_navigation_action_get_target_frame(navigationAction));
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, propertyId, pspec);
        break;
    }
}

static void webkit_web_navigation_action_set_property(GObject* object, guint propertyId, const GValue* value, GParamSpec* pspec)
{
    WebKitWebNavigationAction* navigationAction = WEBKIT_WEB_NAVIGATION_ACTION(object);
    WebKitWebNavigationActionPrivate* priv = navigationAction->priv;

    switch(propertyId) {
    case PROP_REASON:
        webkit_web_navigation_action_set_reason(navigationAction, (WebKitWebNavigationReason)g_value_get_enum(value));
        break;
    case PROP_ORIGINAL_URI:
        webkit_web_navigation_action_set_original_uri(navigationAction, g_value_get_string(value));
        break;
    case PROP_BUTTON:
        priv->button = g_value_get_int(value);
        break;
    case PROP_MODIFIER_STATE:
        priv->modifier_state = g_value_get_int(value);
        break;
    case PROP_TARGET_FRAME:
        webkit_web_navigation_action_set_target_frame(navigationAction, g_value_get_string(value));
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, propertyId, pspec);
        break;
    }
}

static void webkit_web_navigation_action_init(WebKitWebNavigationAction* navigationAction)
{
    navigationAction->priv = WEBKIT_WEB_NAVIGATION_ACTION_GET_PRIVATE(navigationAction);
}

static void webkit_web_navigation_action_finalize(GObject* obj)
{
    WebKitWebNavigationAction* navigationAction = WEBKIT_WEB_NAVIGATION_ACTION(obj);
    WebKitWebNavigationActionPrivate* priv = navigationAction->priv;

    g_free(priv->originalUri);

    G_OBJECT_CLASS(webkit_web_navigation_action_parent_class)->finalize(obj);
}

static void webkit_web_navigation_action_class_init(WebKitWebNavigationActionClass* requestClass)
{
    GObjectClass* objectClass = G_OBJECT_CLASS(requestClass);

    COMPILE_ASSERT(static_cast<int>(WEBKIT_WEB_NAVIGATION_REASON_LINK_CLICKED) == static_cast<int>(WebCore::NavigationTypeLinkClicked), navigation_type_link_clicked_enum_match);
    COMPILE_ASSERT(static_cast<int>(WEBKIT_WEB_NAVIGATION_REASON_FORM_SUBMITTED) == static_cast<int>(WebCore::NavigationTypeFormSubmitted), navigation_type_form_submitted_enum_match);
    COMPILE_ASSERT(static_cast<int>(WEBKIT_WEB_NAVIGATION_REASON_BACK_FORWARD) == static_cast<int>(WebCore::NavigationTypeBackForward), navigation_type_back_forward_enum_match);
    COMPILE_ASSERT(static_cast<int>(WEBKIT_WEB_NAVIGATION_REASON_RELOAD) == static_cast<int>(WebCore::NavigationTypeReload), navigation_type_reload_enum_match);
    COMPILE_ASSERT(static_cast<int>(WEBKIT_WEB_NAVIGATION_REASON_FORM_RESUBMITTED) == static_cast<int>(WebCore::NavigationTypeFormResubmitted), navigation_type_form_resubmitted_enum_match);
    COMPILE_ASSERT(static_cast<int>(WEBKIT_WEB_NAVIGATION_REASON_OTHER) == static_cast<int>(WebCore::NavigationTypeOther), navigation_type_other_enum_match);

    objectClass->get_property = webkit_web_navigation_action_get_property;
    objectClass->set_property = webkit_web_navigation_action_set_property;
    objectClass->dispose = webkit_web_navigation_action_finalize;

    /**
     * WebKitWebNavigationAction:reason:
     *
     * The reason why this navigation is occuring.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(objectClass, PROP_REASON,
                                    g_param_spec_enum("reason",
                                                      _("Reason"),
                                                      _("The reason why this navigation is occurring"),
                                                      WEBKIT_TYPE_WEB_NAVIGATION_REASON,
                                                      WEBKIT_WEB_NAVIGATION_REASON_OTHER,
                                                      (GParamFlags)(WEBKIT_PARAM_READWRITE | G_PARAM_CONSTRUCT)));

    /**
     * WebKitWebNavigationAction:original-uri:
     *
     * The URI that was requested as the target for the navigation.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(objectClass, PROP_ORIGINAL_URI,
                                    g_param_spec_string("original-uri",
                                                        _("Original URI"),
                                                        _("The URI that was requested as the target for the navigation"),
                                                        "",
                                                        (GParamFlags)(WEBKIT_PARAM_READWRITE | G_PARAM_CONSTRUCT)));
    /**
     * WebKitWebNavigationAction:button:
     *
     * The DOM identifier for the mouse button used to click. DOM button values
     * are 0, 1 and 2 for left, middle and right buttons. If the action was not
     * initiated by a mouse click the value will be -1.
     *
     * Since: 1.0.3
     */
    g_object_class_install_property(objectClass, PROP_BUTTON,
                                    g_param_spec_int("button",
                                                     _("Button"),
                                                     _("The button used to click"),
                                                     -1,
                                                     G_MAXINT,
                                                     -1,
                                                     (GParamFlags)(WEBKIT_PARAM_READWRITE | G_PARAM_CONSTRUCT_ONLY)));

    /**
     * WebKitWebNavigationAction:modifier-state:
     *
     * The state of the modifier keys when the action was requested.
     * 
     * Since: 1.0.3
     */
    g_object_class_install_property(objectClass, PROP_MODIFIER_STATE,
                                    g_param_spec_int("modifier-state",
                                                     _("Modifier state"),
                                                     _("A bitmask representing the state of the modifier keys"),
                                                     0,
                                                     G_MAXINT,
                                                     0,
                                                     (GParamFlags)(WEBKIT_PARAM_READWRITE | G_PARAM_CONSTRUCT_ONLY)));

    /**
     * WebKitWebNavigationAction:target-frame:
     *
     * The target frame for the navigation.
     *
     * Since: 1.1.13
     */
    g_object_class_install_property(objectClass, PROP_TARGET_FRAME,
                                    g_param_spec_string("target-frame",
                                                        _("Target frame"),
                                                        _("The target frame for the navigation"),
                                                        NULL,
                                                        (GParamFlags)(WEBKIT_PARAM_READWRITE | G_PARAM_CONSTRUCT_ONLY)));



    g_type_class_add_private(requestClass, sizeof(WebKitWebNavigationActionPrivate));
}

WebKitWebNavigationReason webkit_web_navigation_action_get_reason(WebKitWebNavigationAction* navigationAction)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_NAVIGATION_ACTION(navigationAction), WEBKIT_WEB_NAVIGATION_REASON_OTHER);

    return navigationAction->priv->reason;
}

void webkit_web_navigation_action_set_reason(WebKitWebNavigationAction* navigationAction, WebKitWebNavigationReason reason)
{
    g_return_if_fail(WEBKIT_IS_WEB_NAVIGATION_ACTION(navigationAction));

    if (navigationAction->priv->reason == reason)
        return;

    navigationAction->priv->reason = reason;
    g_object_notify(G_OBJECT(navigationAction), "reason");
}

const gchar* webkit_web_navigation_action_get_original_uri(WebKitWebNavigationAction* navigationAction)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_NAVIGATION_ACTION(navigationAction), NULL);

    return navigationAction->priv->originalUri;
}

void webkit_web_navigation_action_set_original_uri(WebKitWebNavigationAction* navigationAction, const gchar* originalUri)
{
    g_return_if_fail(WEBKIT_IS_WEB_NAVIGATION_ACTION(navigationAction));
    g_return_if_fail(originalUri);

    if (navigationAction->priv->originalUri &&
        (!strcmp(navigationAction->priv->originalUri, originalUri)))
        return;

    g_free(navigationAction->priv->originalUri);
    navigationAction->priv->originalUri = g_strdup(originalUri);
    g_object_notify(G_OBJECT(navigationAction), "original-uri");
}

gint webkit_web_navigation_action_get_button(WebKitWebNavigationAction* navigationAction)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_NAVIGATION_ACTION(navigationAction), -1);

    return navigationAction->priv->button;
}

gint webkit_web_navigation_action_get_modifier_state(WebKitWebNavigationAction* navigationAction)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_NAVIGATION_ACTION(navigationAction), 0);

    return navigationAction->priv->modifier_state;
}

G_CONST_RETURN gchar* webkit_web_navigation_action_get_target_frame(WebKitWebNavigationAction* navigationAction)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_NAVIGATION_ACTION(navigationAction), NULL);

    return navigationAction->priv->targetFrame;
}

static void webkit_web_navigation_action_set_target_frame(WebKitWebNavigationAction* navigationAction, const gchar* targetFrame)
{
    if (!g_strcmp0(navigationAction->priv->targetFrame, targetFrame))
        return;

    g_free(navigationAction->priv->targetFrame);
    navigationAction->priv->targetFrame = g_strdup(targetFrame);
    g_object_notify(G_OBJECT(navigationAction), "target-frame");
}
