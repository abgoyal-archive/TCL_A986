

#ifndef V8NodeFilterCondition_h
#define V8NodeFilterCondition_h

#include "NodeFilterCondition.h"
#include <v8.h>

// NodeFilter is a JavaScript function that takes a Node as parameter and returns a short (ACCEPT, SKIP, REJECT) as the result.
namespace WebCore {

    class Node;
    class ScriptState;

    // NodeFilterCondition is a wrapper around a NodeFilter JS function.
    class V8NodeFilterCondition : public NodeFilterCondition {
    public:
        explicit V8NodeFilterCondition(v8::Handle<v8::Value> filter);
        virtual ~V8NodeFilterCondition();

        virtual short acceptNode(ScriptState*, Node*) const;

    private:
        mutable v8::Persistent<v8::Value> m_filter;
    };

} // namespace WebCore

#endif // V8NodeFilterCondition_h
