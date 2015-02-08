

#ifndef QWEBHISTORYINTERFACE_H
#define QWEBHISTORYINTERFACE_H

#include <QtCore/qobject.h>

#include "qwebkitglobal.h"

class QWEBKIT_EXPORT QWebHistoryInterface : public QObject {
    Q_OBJECT
public:
    QWebHistoryInterface(QObject *parent = 0);
    ~QWebHistoryInterface();

    static void setDefaultInterface(QWebHistoryInterface *defaultInterface);
    static QWebHistoryInterface *defaultInterface();

    virtual bool historyContains(const QString &url) const = 0;
    virtual void addHistoryEntry(const QString &url) = 0;
};

#endif
