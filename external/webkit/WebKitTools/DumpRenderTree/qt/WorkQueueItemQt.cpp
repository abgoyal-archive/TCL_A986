
#include "config.h"
#include "WorkQueueItemQt.h"

QWebFrame* findFrameNamed(const QString& frameName, QWebFrame* frame)
{
    if (frame->frameName() == frameName)
        return frame;

    foreach (QWebFrame* childFrame, frame->childFrames())
        if (QWebFrame* f = findFrameNamed(frameName, childFrame))
            return f;

    return 0;
}

bool LoadItem::invoke() const
{
    //qDebug() << ">>>LoadItem::invoke";
    Q_ASSERT(m_webPage);

    QWebFrame* frame = 0;
    const QString t = target();
    if (t.isEmpty())
        frame = m_webPage->mainFrame();
    else
        frame = findFrameNamed(t, m_webPage->mainFrame());

    if (!frame)
        return false;

    frame->load(url());
    return true;
}

bool ReloadItem::invoke() const
{
    //qDebug() << ">>>ReloadItem::invoke";
    Q_ASSERT(m_webPage);
    m_webPage->triggerAction(QWebPage::Reload);
    return true;
}

bool ScriptItem::invoke() const
{
    //qDebug() << ">>>ScriptItem::invoke";
    Q_ASSERT(m_webPage);
    m_webPage->mainFrame()->evaluateJavaScript(script());
    return true;
}

bool BackForwardItem::invoke() const
{
    //qDebug() << ">>>BackForwardItem::invoke";
    Q_ASSERT(m_webPage);
    if (!m_howFar)
        return false;

    if (m_howFar > 0) {
        for (int i = 0; i != m_howFar; ++i)
            m_webPage->triggerAction(QWebPage::Forward);
    } else {
        for (int i = 0; i != m_howFar; --i)
            m_webPage->triggerAction(QWebPage::Back);
    }
    return true;
}
