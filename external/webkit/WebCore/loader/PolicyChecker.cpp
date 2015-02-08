

#include "config.h"
#include "PolicyChecker.h"

#include "DocumentLoader.h"
#include "FormState.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "FrameLoaderClient.h"
#include "HTMLFormElement.h"

namespace WebCore {

PolicyChecker::PolicyChecker(Frame* frame)
    : m_frame(frame)
    , m_delegateIsDecidingNavigationPolicy(false)
    , m_delegateIsHandlingUnimplementablePolicy(false)
    , m_loadType(FrameLoadTypeStandard)
{
}

void PolicyChecker::checkNavigationPolicy(const ResourceRequest& newRequest, NavigationPolicyDecisionFunction function, void* argument)
{
    checkNavigationPolicy(newRequest, m_frame->loader()->activeDocumentLoader(), 0, function, argument);
}

void PolicyChecker::checkNavigationPolicy(const ResourceRequest& request, DocumentLoader* loader,
    PassRefPtr<FormState> formState, NavigationPolicyDecisionFunction function, void* argument)
{
    NavigationAction action = loader->triggeringAction();
    if (action.isEmpty()) {
        action = NavigationAction(request.url(), NavigationTypeOther);
        loader->setTriggeringAction(action);
    }

    // Don't ask more than once for the same request or if we are loading an empty URL.
    // This avoids confusion on the part of the client.
    if (equalIgnoringHeaderFields(request, loader->lastCheckedRequest()) || (!request.isNull() && request.url().isEmpty())) {
        function(argument, request, 0, true);
        loader->setLastCheckedRequest(request);
        return;
    }
    
    // We are always willing to show alternate content for unreachable URLs;
    // treat it like a reload so it maintains the right state for b/f list.
    if (loader->substituteData().isValid() && !loader->substituteData().failingURL().isEmpty()) {
        if (isBackForwardLoadType(m_loadType))
            m_loadType = FrameLoadTypeReload;
        function(argument, request, 0, true);
        return;
    }
    
    loader->setLastCheckedRequest(request);

    m_callback.set(request, formState.get(), function, argument);

    m_delegateIsDecidingNavigationPolicy = true;
    m_frame->loader()->client()->dispatchDecidePolicyForNavigationAction(&PolicyChecker::continueAfterNavigationPolicy,
        action, request, formState);
    m_delegateIsDecidingNavigationPolicy = false;
}

void PolicyChecker::checkNewWindowPolicy(const NavigationAction& action, NewWindowPolicyDecisionFunction function,
    const ResourceRequest& request, PassRefPtr<FormState> formState, const String& frameName, void* argument)
{
    m_callback.set(request, formState, frameName, function, argument);
    m_frame->loader()->client()->dispatchDecidePolicyForNewWindowAction(&PolicyChecker::continueAfterNewWindowPolicy,
        action, request, formState, frameName);
}

void PolicyChecker::checkContentPolicy(const String& MIMEType, ContentPolicyDecisionFunction function, void* argument)
{
    m_callback.set(function, argument);
    m_frame->loader()->client()->dispatchDecidePolicyForMIMEType(&PolicyChecker::continueAfterContentPolicy,
        MIMEType, m_frame->loader()->activeDocumentLoader()->request());
}

void PolicyChecker::cancelCheck()
{
    m_frame->loader()->client()->cancelPolicyCheck();
    m_callback.clear();
}

void PolicyChecker::stopCheck()
{
    m_frame->loader()->client()->cancelPolicyCheck();
    PolicyCallback callback = m_callback;
    m_callback.clear();
    callback.cancel();
}

void PolicyChecker::cannotShowMIMEType(const ResourceResponse& response)
{
    handleUnimplementablePolicy(m_frame->loader()->client()->cannotShowMIMETypeError(response));
}

void PolicyChecker::continueLoadAfterWillSubmitForm(PolicyAction)
{
    // See header file for an explaination of why this function
    // isn't like the others.
    m_frame->loader()->continueLoadAfterWillSubmitForm();
}

void PolicyChecker::continueAfterNavigationPolicy(PolicyAction policy)
{
    PolicyCallback callback = m_callback;
    m_callback.clear();

    bool shouldContinue = policy == PolicyUse;

    switch (policy) {
        case PolicyIgnore:
            callback.clearRequest();
            break;
        case PolicyDownload:
            m_frame->loader()->client()->startDownload(callback.request());
            callback.clearRequest();
            break;
        case PolicyUse: {
            ResourceRequest request(callback.request());

            if (!m_frame->loader()->client()->canHandleRequest(request)) {
                handleUnimplementablePolicy(m_frame->loader()->cannotShowURLError(callback.request()));
                callback.clearRequest();
                shouldContinue = false;
            }
            break;
        }
    }

    callback.call(shouldContinue);
}

void PolicyChecker::continueAfterNewWindowPolicy(PolicyAction policy)
{
    PolicyCallback callback = m_callback;
    m_callback.clear();

    switch (policy) {
        case PolicyIgnore:
            callback.clearRequest();
            break;
        case PolicyDownload:
            m_frame->loader()->client()->startDownload(callback.request());
            callback.clearRequest();
            break;
        case PolicyUse:
            break;
    }

    callback.call(policy == PolicyUse);
}

void PolicyChecker::continueAfterContentPolicy(PolicyAction policy)
{
    PolicyCallback callback = m_callback;
    m_callback.clear();
    callback.call(policy);
}

void PolicyChecker::handleUnimplementablePolicy(const ResourceError& error)
{
    m_delegateIsHandlingUnimplementablePolicy = true;
    m_frame->loader()->client()->dispatchUnableToImplementPolicy(error);
    m_delegateIsHandlingUnimplementablePolicy = false;
}

} // namespace WebCore
