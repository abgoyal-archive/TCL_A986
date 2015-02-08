

#ifndef V8BindingState_h
#define V8BindingState_h

#include "GenericBinding.h"
#include "V8Binding.h"

namespace WebCore {

// Singleton implementation of State<V8Binding>.  Uses V8's global data
// structures to return information about relevant execution state.
template <>
class State<V8Binding> : public State<GenericBinding> {
public:
    // Singleton
    static State* Only();

    // The DOMWindow corresponding to the 'calling context' of execution.
    DOMWindow* getActiveWindow();

    // Reports an error message (without delay) if the security check fails.
    static void immediatelyReportUnsafeAccessTo(Frame*);

private:
    explicit State() {}
    ~State();
};

typedef State<V8Binding> V8BindingState;

}

#endif // V8BindingState_h
