

#ifndef PolicyChecker_h
#define PolicyChecker_h

#include "FrameLoaderTypes.h"
#include "PlatformString.h"
#include "PolicyCallback.h"
#include "ResourceRequest.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

class DocumentLoader;
class FormState;
class Frame;
class NavigationAction;
class ResourceError;
class ResourceResponse;

class PolicyChecker : public Noncopyable {
public:
    PolicyChecker(Frame*);

    void checkNavigationPolicy(const ResourceRequest&, DocumentLoader*, PassRefPtr<FormState>, NavigationPolicyDecisionFunction, void* argument);
    void checkNavigationPolicy(const ResourceRequest&, NavigationPolicyDecisionFunction, void* argument);
    void checkNewWindowPolicy(const NavigationAction&, NewWindowPolicyDecisionFunction, const ResourceRequest&, PassRefPtr<FormState>, const String& frameName, void* argument);
    void checkContentPolicy(const String& MIMEType, ContentPolicyDecisionFunction, void* argument);

    // FIXME: These are different.  They could use better names.
    void cancelCheck();
    void stopCheck();

    void cannotShowMIMEType(const ResourceResponse&);

    FrameLoadType loadType() const { return m_loadType; }
    void setLoadType(FrameLoadType loadType) { m_loadType = loadType; }

    bool delegateIsDecidingNavigationPolicy() const { return m_delegateIsDecidingNavigationPolicy; }
    bool delegateIsHandlingUnimplementablePolicy() const { return m_delegateIsHandlingUnimplementablePolicy; }

    // FIXME: This function is a cheat.  Basically, this is just an asynchronouc callback
    // from the FrameLoaderClient, but this callback uses the policy types and so has to
    // live on this object.  In the long term, we should create a type for non-policy
    // callbacks from the FrameLoaderClient and remove this vestige.  I just don't have
    // the heart to hack on all the platforms to make that happen right now.
    void continueLoadAfterWillSubmitForm(PolicyAction);

private:
    void continueAfterNavigationPolicy(PolicyAction);
    void continueAfterNewWindowPolicy(PolicyAction);
    void continueAfterContentPolicy(PolicyAction);

    void handleUnimplementablePolicy(const ResourceError&);

    Frame* m_frame;

    bool m_delegateIsDecidingNavigationPolicy;
    bool m_delegateIsHandlingUnimplementablePolicy;

    // This identifies the type of navigation action which prompted this load. Note 
    // that WebKit conveys this value as the WebActionNavigationTypeKey value
    // on navigation action delegate callbacks.
    FrameLoadType m_loadType;
    PolicyCallback m_callback;
};

} // namespace WebCore

#endif // PolicyChecker_h
