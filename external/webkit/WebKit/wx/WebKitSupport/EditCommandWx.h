
 
#ifndef WXEDITCOMMAND_H
#define WXEDITCOMMAND_H

#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {
    class EditCommand;
}

class EditCommandWx 
{
public:
    EditCommandWx(WTF::PassRefPtr<WebCore::EditCommand> command) 
    {
        m_editCommand = command;
    }
    
    ~EditCommandWx() {}
    WTF::PassRefPtr<WebCore::EditCommand> editCommand() { return m_editCommand; }
    
private:
    WTF::RefPtr<WebCore::EditCommand> m_editCommand;
};

#endif
