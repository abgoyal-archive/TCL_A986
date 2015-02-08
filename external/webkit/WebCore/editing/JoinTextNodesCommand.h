

#ifndef JoinTextNodesCommand_h
#define JoinTextNodesCommand_h

#include "EditCommand.h"

namespace WebCore {

class Text;

class JoinTextNodesCommand : public SimpleEditCommand {
public:
    static PassRefPtr<JoinTextNodesCommand> create(PassRefPtr<Text> text1, PassRefPtr<Text> text2)
    {
        return adoptRef(new JoinTextNodesCommand(text1, text2));
    }

private:
    JoinTextNodesCommand(PassRefPtr<Text>, PassRefPtr<Text>);

    virtual void doApply();
    virtual void doUnapply();

    RefPtr<Text> m_text1;
    RefPtr<Text> m_text2;
};

} // namespace WebCore

#endif // JoinTextNodesCommand_h
