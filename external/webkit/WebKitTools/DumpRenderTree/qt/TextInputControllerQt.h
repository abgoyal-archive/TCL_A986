
#ifndef TextInputControllerQt_h
#define TextInputControllerQt_h

#include <QList>
#include <QObject>
#include <QString>
#include "qwebpage.h"

class TextInputController : public QObject {
    Q_OBJECT
public:
    TextInputController(QWebPage* parent);

public slots:
    void doCommand(const QString& command);
//     void setMarkedText(const QString& str, int from, int length);
//     bool hasMarkedText();
//     void unmarkText();
//     QList<int> markedRange();
//     QList<int> selectedRange();
//     void validAttributesForMarkedText();
//     void inserText(const QString&);
//     void firstRectForCharacterRange();
//     void characterIndexForPoint(int, int);
//     void substringFromRange(int, int);
//     void conversationIdentifier();
};
#endif // TextInputControllerQt_h
