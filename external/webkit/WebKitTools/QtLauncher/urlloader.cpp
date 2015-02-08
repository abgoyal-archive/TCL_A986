

#include "urlloader.h"

#include <QFile>
#include <QDebug>

UrlLoader::UrlLoader(QWebFrame* frame, const QString& inputFileName)
    : m_frame(frame)
    , m_stdOut(stdout)
    , m_loaded(0)
{
    init(inputFileName);
}

void UrlLoader::loadNext()
{
    QString qstr;
    if (getUrl(qstr)) {
        QUrl url(qstr, QUrl::StrictMode);
        if (url.isValid()) {
            m_stdOut << "Loading " << qstr << " ......" << ++m_loaded << endl;
            m_frame->load(url);
        } else
            loadNext();
    } else
        disconnect(m_frame, 0, this, 0);
}

void UrlLoader::init(const QString& inputFileName)
{
    QFile inputFile(inputFileName);
    if (inputFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream stream(&inputFile);
        QString line;
        while (true) {
            line = stream.readLine();
            if (line.isNull())
                break;
            m_urls.append(line);
        }
    } else {
        qDebug() << "Can't open list file";
        exit(0);
    }
    m_index = 0;
    inputFile.close();
}

bool UrlLoader::getUrl(QString& qstr)
{
    if (m_index == m_urls.size())
        return false;

    qstr = m_urls[m_index++];
    return true;
}
