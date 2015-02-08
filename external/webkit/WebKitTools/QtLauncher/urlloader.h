

#ifndef urlloader_h
#define urlloader_h

#include "qwebframe.h"

#include <QTextStream>
#include <QVector>

class UrlLoader : public QObject {
    Q_OBJECT

public:
    UrlLoader(QWebFrame* frame, const QString& inputFileName);

public slots:
    void loadNext();

private:
    void init(const QString& inputFileName);
    bool getUrl(QString& qstr);

private:
    QVector<QString> m_urls;
    int m_index;
    QWebFrame* m_frame;
    QTextStream m_stdOut;
    int m_loaded;
};

#endif
