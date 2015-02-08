

#include "config.h"
#include <wtf/Platform.h>
#include "EditCommandQt.h"

using namespace WebCore;

EditCommandQt::EditCommandQt(WTF::RefPtr<EditCommand> cmd, QUndoCommand *parent)
:
#ifndef QT_NO_UNDOCOMMAND
    QUndoCommand(parent),
#endif
    _cmd(cmd), _first(true)
{
}


EditCommandQt::~EditCommandQt() {
}


void EditCommandQt::redo() {
    if (_first) {
        _first = false;
        return;
    }
    if (_cmd) {
        _cmd->reapply();
    }
}


void EditCommandQt::undo() {
    if (_cmd) {
        _cmd->unapply();
    }
}


// vim: ts=4 sw=4 et
