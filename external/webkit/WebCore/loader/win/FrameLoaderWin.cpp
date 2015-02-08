

#include "config.h"
#include "FrameLoader.h"

#include "DocumentLoader.h"
#include "FrameLoadRequest.h"
#include "FrameWin.h"
#include "ResourceRequest.h"

namespace WebCore {

void FrameLoader::urlSelected(const FrameLoadRequest& request, Event* /*triggering Event*/)
{
    FrameWin* frameWin = static_cast<FrameWin*>(m_frame);
    if (frameWin->client())
        frameWin->client()->openURL(request.resourceRequest().url().string(), request.lockHistory());
}

void FrameLoader::submitForm(const FrameLoadRequest& request, Event*)
{
    const ResourceRequest& resourceRequest = request.resourceRequest();

#ifdef MULTIPLE_FORM_SUBMISSION_PROTECTION
    // FIXME: this is a hack inherited from FrameMac, and should be pushed into Frame
    if (m_submittedFormURL == resourceRequest.url())
        return;
    m_submittedFormURL = resourceRequest.url();
#endif

    FrameWin* frameWin = static_cast<FrameWin*>(m_frame);
    if (frameWin->client())
        frameWin->client()->submitForm(resourceRequest.httpMethod(), resourceRequest.url(), resourceRequest.httpBody());

    clearRecordedFormValues();
}

}
