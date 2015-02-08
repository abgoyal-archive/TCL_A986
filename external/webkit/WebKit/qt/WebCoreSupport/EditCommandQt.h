

#ifndef EDITCOMMANDQT_H
#define EDITCOMMANDQT_H

#include <qglobal.h>
QT_BEGIN_NAMESPACE
class QUndoCommand;
QT_END_NAMESPACE

#include <QUndoCommand>
#include <EditCommand.h>

class EditCommandQt
#ifndef QT_NO_UNDOCOMMAND
    : public QUndoCommand
#endif
{
    public:
        EditCommandQt(WTF::RefPtr<WebCore::EditCommand> cmd, QUndoCommand *parent = 0);
        ~EditCommandQt();

        void redo();
        void undo();

    private:
        WTF::RefPtr<WebCore::EditCommand> _cmd;
        bool _first;
};

#endif

// vim: ts=4 sw=4 et
