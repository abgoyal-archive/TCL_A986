

#ifndef PolicyCallback_h
#define PolicyCallback_h

#include "FrameLoaderTypes.h"
#include "PlatformString.h"
#include "ResourceRequest.h"
#include <wtf/RefPtr.h>

namespace WebCore {

class FormState;

typedef void (*NavigationPolicyDecisionFunction)(void* argument,
    const ResourceRequest&, PassRefPtr<FormState>, bool shouldContinue);
typedef void (*NewWindowPolicyDecisionFunction)(void* argument,
    const ResourceRequest&, PassRefPtr<FormState>, const String& frameName, bool shouldContinue);
typedef void (*ContentPolicyDecisionFunction)(void* argument, PolicyAction);

class PolicyCallback {
public:
    PolicyCallback();
    ~PolicyCallback();

    void clear();
    void set(const ResourceRequest&, PassRefPtr<FormState>,
        NavigationPolicyDecisionFunction, void* argument);
    void set(const ResourceRequest&, PassRefPtr<FormState>, const String& frameName,
        NewWindowPolicyDecisionFunction, void* argument);
    void set(ContentPolicyDecisionFunction, void* argument);

    const ResourceRequest& request() const { return m_request; }
    void clearRequest();

    void call(bool shouldContinue);
    void call(PolicyAction);
    void cancel();

private:
    ResourceRequest m_request;
    RefPtr<FormState> m_formState;
    String m_frameName;

    NavigationPolicyDecisionFunction m_navigationFunction;
    NewWindowPolicyDecisionFunction m_newWindowFunction;
    ContentPolicyDecisionFunction m_contentFunction;
    void* m_argument;
};

} // namespace WebCore

#endif // PolicyCallback_h
