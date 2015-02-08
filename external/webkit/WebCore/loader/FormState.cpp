

#include "config.h"
#include "FormState.h"

#include "Frame.h"
#include "HTMLFormElement.h"

namespace WebCore {

inline FormState::FormState(PassRefPtr<HTMLFormElement> form, StringPairVector& textFieldValuesToAdopt, PassRefPtr<Frame> sourceFrame, FormSubmissionTrigger formSubmissionTrigger)
    : m_form(form)
    , m_sourceFrame(sourceFrame)
    , m_formSubmissionTrigger(formSubmissionTrigger)
{
    m_textFieldValues.swap(textFieldValuesToAdopt);
}

PassRefPtr<FormState> FormState::create(PassRefPtr<HTMLFormElement> form, StringPairVector& textFieldValuesToAdopt, PassRefPtr<Frame> sourceFrame, FormSubmissionTrigger formSubmissionTrigger)
{
    return adoptRef(new FormState(form, textFieldValuesToAdopt, sourceFrame, formSubmissionTrigger));
}

}
