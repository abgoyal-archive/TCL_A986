

#ifndef MessageChannel_h
#define MessageChannel_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class MessagePort;
    class ScriptExecutionContext;

    class MessageChannel : public RefCounted<MessageChannel> {
    public:
        static PassRefPtr<MessageChannel> create(ScriptExecutionContext* context) { return adoptRef(new MessageChannel(context)); }
        ~MessageChannel();

        MessagePort* port1() const { return m_port1.get(); }
        MessagePort* port2() const { return m_port2.get(); }

    private:
        MessageChannel(ScriptExecutionContext*);

        RefPtr<MessagePort> m_port1;
        RefPtr<MessagePort> m_port2;
    };

} // namespace WebCore

#endif // MessageChannel_h
