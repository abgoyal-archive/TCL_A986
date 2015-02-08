

#ifndef FormState_h
#define FormState_h

#include "PlatformString.h"

namespace WebCore {

    class Frame;
    class HTMLFormElement;

    enum FormSubmissionTrigger {
        SubmittedByJavaScript,
        NotSubmittedByJavaScript
    };
    
    typedef Vector<std::pair<String, String> > StringPairVector;

    class FormState : public RefCounted<FormState> {
    public:
        static PassRefPtr<FormState> create(PassRefPtr<HTMLFormElement>, StringPairVector& textFieldValuesToAdopt, PassRefPtr<Frame>, FormSubmissionTrigger);

        HTMLFormElement* form() const { return m_form.get(); }
        const StringPairVector& textFieldValues() const { return m_textFieldValues; }
        Frame* sourceFrame() const { return m_sourceFrame.get(); }
        FormSubmissionTrigger formSubmissionTrigger() const { return m_formSubmissionTrigger; }

    private:
        FormState(PassRefPtr<HTMLFormElement>, StringPairVector& textFieldValuesToAdopt, PassRefPtr<Frame>, FormSubmissionTrigger);

        RefPtr<HTMLFormElement> m_form;
        StringPairVector m_textFieldValues;
        RefPtr<Frame> m_sourceFrame;
        FormSubmissionTrigger m_formSubmissionTrigger;
    };

}

#endif // FormState_h
