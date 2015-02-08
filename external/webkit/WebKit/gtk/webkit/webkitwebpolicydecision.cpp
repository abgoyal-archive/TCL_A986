

#include "config.h"

#include "webkitwebpolicydecision.h"

#include "FrameLoaderTypes.h"
#include "webkitprivate.h"

using namespace WebKit;
using namespace WebCore;


G_DEFINE_TYPE(WebKitWebPolicyDecision, webkit_web_policy_decision, G_TYPE_OBJECT);

struct _WebKitWebPolicyDecisionPrivate {
    WebKitWebFrame* frame;
    FramePolicyFunction framePolicyFunction;
    gboolean isCancelled;
};

#define WEBKIT_WEB_POLICY_DECISION_GET_PRIVATE(obj)    (G_TYPE_INSTANCE_GET_PRIVATE((obj), WEBKIT_TYPE_WEB_POLICY_DECISION, WebKitWebPolicyDecisionPrivate))

static void webkit_web_policy_decision_class_init(WebKitWebPolicyDecisionClass* decisionClass)
{
    g_type_class_add_private(decisionClass, sizeof(WebKitWebPolicyDecisionPrivate));
}

static void webkit_web_policy_decision_init(WebKitWebPolicyDecision* decision)
{
    decision->priv = WEBKIT_WEB_POLICY_DECISION_GET_PRIVATE(decision);
}

WebKitWebPolicyDecision* webkit_web_policy_decision_new(WebKitWebFrame* frame, WebCore::FramePolicyFunction function)
{
    g_return_val_if_fail(frame, NULL);

    WebKitWebPolicyDecision* decision = WEBKIT_WEB_POLICY_DECISION(g_object_new(WEBKIT_TYPE_WEB_POLICY_DECISION, NULL));
    WebKitWebPolicyDecisionPrivate* priv = decision->priv;

    priv->frame = frame;
    priv->framePolicyFunction = function;
    priv->isCancelled = FALSE;

    return decision;
}

void webkit_web_policy_decision_use(WebKitWebPolicyDecision* decision)
{
    g_return_if_fail(WEBKIT_IS_WEB_POLICY_DECISION(decision));

    WebKitWebPolicyDecisionPrivate* priv = decision->priv;

    if (!priv->isCancelled)
        (core(priv->frame)->loader()->policyChecker()->*(priv->framePolicyFunction))(WebCore::PolicyUse);
}

void webkit_web_policy_decision_ignore(WebKitWebPolicyDecision* decision)
{
    g_return_if_fail(WEBKIT_IS_WEB_POLICY_DECISION(decision));

    WebKitWebPolicyDecisionPrivate* priv = decision->priv;

    if (!priv->isCancelled)
        (core(priv->frame)->loader()->policyChecker()->*(priv->framePolicyFunction))(WebCore::PolicyIgnore);
}

void webkit_web_policy_decision_download(WebKitWebPolicyDecision* decision)
{
    g_return_if_fail(WEBKIT_IS_WEB_POLICY_DECISION(decision));

    WebKitWebPolicyDecisionPrivate* priv = decision->priv;

    if (!priv->isCancelled)
        (core(priv->frame)->loader()->policyChecker()->*(priv->framePolicyFunction))(WebCore::PolicyDownload);
}

void webkit_web_policy_decision_cancel(WebKitWebPolicyDecision* decision)
{
    g_return_if_fail(WEBKIT_IS_WEB_POLICY_DECISION(decision));

    WebKitWebPolicyDecisionPrivate* priv = decision->priv;

    priv->isCancelled = TRUE;
}
