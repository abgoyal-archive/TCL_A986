

#ifndef WorkQueueItemQt_h
#define WorkQueueItemQt_h

#include <QPointer>
#include <QString>
#include <qwebframe.h>
#include <qwebpage.h>

class WorkQueueItem {
public:
    WorkQueueItem(QWebPage *page) : m_webPage(page) {}
    virtual ~WorkQueueItem() { }
    virtual bool invoke() const = 0;

protected:
    QPointer<QWebPage> m_webPage;
};

class LoadItem : public WorkQueueItem {
public:
    LoadItem(const QString &url, const QString &target, QWebPage *page)
        : WorkQueueItem(page)
        , m_url(url)
        , m_target(target)
    {
    }

    QString url() const { return m_url; }
    QString target() const { return m_target; }

    virtual bool invoke() const;

private:
    QString m_url;
    QString m_target;
};

class ReloadItem : public WorkQueueItem {
public:
    ReloadItem(QWebPage *page)
        : WorkQueueItem(page)
    {
    }
    virtual bool invoke() const;
};

class ScriptItem : public WorkQueueItem {
public:
    ScriptItem(const QString &script, QWebPage *page)
        : WorkQueueItem(page)
        , m_script(script)
    {
    }

    QString script() const { return m_script; }

    virtual bool invoke() const;

private:
    QString m_script;
};

class LoadingScriptItem : public ScriptItem {
public:
    LoadingScriptItem(const QString& script, QWebPage* page)
        : ScriptItem(script, page)
    {
    }

    virtual bool invoke() const { return ScriptItem::invoke(); }
};

class NonLoadingScriptItem : public ScriptItem {
public:
    NonLoadingScriptItem(const QString& script, QWebPage* page)
        : ScriptItem(script, page)
    {
    }

    virtual bool invoke() const { ScriptItem::invoke(); return false; }
};


class BackForwardItem : public WorkQueueItem {
public:
    virtual bool invoke() const;

protected:
    BackForwardItem(int howFar, QWebPage *page)
        : WorkQueueItem(page)
        , m_howFar(howFar)
    {
    }

    int m_howFar;
};

class BackItem : public BackForwardItem {
public:
    BackItem(unsigned howFar, QWebPage *page)
        : BackForwardItem(-howFar, page)
    {
    }
};

class ForwardItem : public BackForwardItem {
public:
    ForwardItem(unsigned howFar, QWebPage *page)
        : BackForwardItem(howFar, page)
    {
    }
};

#endif // !defined(WorkQueueItemQt_h)
