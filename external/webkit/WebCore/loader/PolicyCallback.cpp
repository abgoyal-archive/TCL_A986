

#include "config.h"
#include "PolicyCallback.h"

#include "FormState.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "HTMLFormElement.h"

namespace WebCore {

PolicyCallback::PolicyCallback()
    : m_navigationFunction(0)
    , m_newWindowFunction(0)
    , m_contentFunction(0)
{
}

PolicyCallback::~PolicyCallback()
{
}

void PolicyCallback::clear()
{
    clearRequest();
    m_navigationFunction = 0;
    m_newWindowFunction = 0;
    m_contentFunction = 0;
}

void PolicyCallback::set(const ResourceRequest& request, PassRefPtr<FormState> formState,
    NavigationPolicyDecisionFunction function, void* argument)
{
    m_request = request;
    m_formState = formState;
    m_frameName = String();

    m_navigationFunction = function;
    m_newWindowFunction = 0;
    m_contentFunction = 0;
    m_argument = argument;
}

void PolicyCallback::set(const ResourceRequest& request, PassRefPtr<FormState> formState,
    const String& frameName, NewWindowPolicyDecisionFunction function, void* argument)
{
    m_request = request;
    m_formState = formState;
    m_frameName = frameName;

    m_navigationFunction = 0;
    m_newWindowFunction = function;
    m_contentFunction = 0;
    m_argument = argument;
}

void PolicyCallback::set(ContentPolicyDecisionFunction function, void* argument)
{
    m_request = ResourceRequest();
    m_formState = 0;
    m_frameName = String();

    m_navigationFunction = 0;
    m_newWindowFunction = 0;
    m_contentFunction = function;
    m_argument = argument;
}

void PolicyCallback::call(bool shouldContinue)
{
    if (m_navigationFunction)
        m_navigationFunction(m_argument, m_request, m_formState.get(), shouldContinue);
    if (m_newWindowFunction)
        m_newWindowFunction(m_argument, m_request, m_formState.get(), m_frameName, shouldContinue);
    ASSERT(!m_contentFunction);
}

void PolicyCallback::call(PolicyAction action)
{
    ASSERT(!m_navigationFunction);
    ASSERT(!m_newWindowFunction);
    ASSERT(m_contentFunction);
    m_contentFunction(m_argument, action);
}

void PolicyCallback::clearRequest()
{
    m_request = ResourceRequest();
    m_formState = 0;
    m_frameName = String();
}

void PolicyCallback::cancel()
{
    clearRequest();
    if (m_navigationFunction)
        m_navigationFunction(m_argument, m_request, m_formState.get(), false);
    if (m_newWindowFunction)
        m_newWindowFunction(m_argument, m_request, m_formState.get(), m_frameName, false);
    if (m_contentFunction)
        m_contentFunction(m_argument, PolicyIgnore);
}

} // namespace WebCore
