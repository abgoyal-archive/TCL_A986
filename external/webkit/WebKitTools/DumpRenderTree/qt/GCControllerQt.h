
#ifndef GCControllerQt_h
#define GCControllerQt_h

#include <QObject>

class QWebPage;

class GCController : public QObject
{
    Q_OBJECT
public:
    GCController(QWebPage* parent);

public slots:
    void collect() const;
    void collectOnAlternateThread(bool waitUntilDone) const;
    size_t getJSObjectCount() const;
};

#endif
