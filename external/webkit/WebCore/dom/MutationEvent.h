

#ifndef MutationEvent_h
#define MutationEvent_h

#include "Event.h"
#include "Node.h"

namespace WebCore {

    class MutationEvent : public Event {
    public:
        enum attrChangeType {
            MODIFICATION    = 1,
            ADDITION        = 2,
            REMOVAL         = 3
        };

        static PassRefPtr<MutationEvent> create()
        {
            return adoptRef(new MutationEvent);
        }

        static PassRefPtr<MutationEvent> create(const AtomicString& type, bool canBubble, PassRefPtr<Node> relatedNode = 0,
            const String& prevValue = String(), const String& newValue = String(), const String& attrName = String(), unsigned short attrChange = 0)
        {
            return adoptRef(new MutationEvent(type, canBubble, false, relatedNode, prevValue, newValue, attrName, attrChange));
        }

        void initMutationEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<Node> relatedNode,
                               const String& prevValue, const String& newValue,
                               const String& attrName, unsigned short attrChange);

        Node* relatedNode() const { return m_relatedNode.get(); }
        String prevValue() const { return m_prevValue; }
        String newValue() const { return m_newValue; }
        String attrName() const { return m_attrName; }
        unsigned short attrChange() const { return m_attrChange; }

        virtual bool isMutationEvent() const;

    private:
        MutationEvent();
        MutationEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<Node> relatedNode,
                      const String& prevValue, const String& newValue,
                      const String& attrName, unsigned short attrChange);

        RefPtr<Node> m_relatedNode;
        String m_prevValue;
        String m_newValue;
        String m_attrName;
        unsigned short m_attrChange;
    };

} // namespace WebCore

#endif // MutationEvent_h
