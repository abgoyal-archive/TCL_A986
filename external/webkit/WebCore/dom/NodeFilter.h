

#ifndef NodeFilter_h
#define NodeFilter_h

#include "NodeFilterCondition.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class NodeFilter : public RefCounted<NodeFilter> {
    public:
        /**
         * The following constants are returned by the acceptNode()
         * method:
         */
        enum {
            FILTER_ACCEPT = 1,
            FILTER_REJECT = 2,
            FILTER_SKIP   = 3
        };

        /**
         * These are the available values for the whatToShow parameter.
         * They are the same as the set of possible types for Node, and
         * their values are derived by using a bit position corresponding
         * to the value of NodeType for the equivalent node type.
         */
        enum {
            SHOW_ALL                       = 0xFFFFFFFF,
            SHOW_ELEMENT                   = 0x00000001,
            SHOW_ATTRIBUTE                 = 0x00000002,
            SHOW_TEXT                      = 0x00000004,
            SHOW_CDATA_SECTION             = 0x00000008,
            SHOW_ENTITY_REFERENCE          = 0x00000010,
            SHOW_ENTITY                    = 0x00000020,
            SHOW_PROCESSING_INSTRUCTION    = 0x00000040,
            SHOW_COMMENT                   = 0x00000080,
            SHOW_DOCUMENT                  = 0x00000100,
            SHOW_DOCUMENT_TYPE             = 0x00000200,
            SHOW_DOCUMENT_FRAGMENT         = 0x00000400,
            SHOW_NOTATION                  = 0x00000800
        };

        static PassRefPtr<NodeFilter> create(PassRefPtr<NodeFilterCondition> condition)
        {
            return adoptRef(new NodeFilter(condition));
        }

        short acceptNode(ScriptState*, Node*) const;
        void markAggregate(JSC::MarkStack& markStack) { m_condition->markAggregate(markStack); };

        // Do not call these functions. They are just scaffolding to support the Objective-C bindings.
        // They operate in the main thread normal world, and they swallow JS exceptions.
        short acceptNode(Node* node) const { return acceptNode(scriptStateFromNode(mainThreadNormalWorld(), node), node); }

    private:
        NodeFilter(PassRefPtr<NodeFilterCondition> condition) : m_condition(condition) { }

        RefPtr<NodeFilterCondition> m_condition;
    };

} // namespace WebCore

#endif // NodeFilter_h
