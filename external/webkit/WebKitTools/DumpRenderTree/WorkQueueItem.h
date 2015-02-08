

#ifndef WorkQueueItem_h
#define WorkQueueItem_h

#include <JavaScriptCore/JSRetainPtr.h>
#include <JavaScriptCore/JSBase.h>

class WorkQueueItem {
public:
    virtual ~WorkQueueItem() { }
    virtual bool invoke() const = 0; // Returns true if this started a load.
};

class LoadItem : public WorkQueueItem {
public:
    LoadItem(const JSStringRef url, const JSStringRef target)
        : m_url(url)
        , m_target(target)
    {
    }

private:
    virtual bool invoke() const;

    JSRetainPtr<JSStringRef> m_url;
    JSRetainPtr<JSStringRef> m_target;
};

class ReloadItem : public WorkQueueItem {
private:
    virtual bool invoke() const;
};

class ScriptItem : public WorkQueueItem {
protected:
    ScriptItem(const JSStringRef script)
        : m_script(script)
    {
    }

protected:
    virtual bool invoke() const;

private:
    JSRetainPtr<JSStringRef> m_script;
};

class LoadingScriptItem : public ScriptItem {
public:
    LoadingScriptItem(const JSStringRef script)
        : ScriptItem(script)
    {
    }

private:
    virtual bool invoke() const { return ScriptItem::invoke(); }
};

class NonLoadingScriptItem : public ScriptItem {
public:
    NonLoadingScriptItem(const JSStringRef script)
        : ScriptItem(script)
    {
    }

private:
    virtual bool invoke() const { ScriptItem::invoke(); return false; }
};

class BackForwardItem : public WorkQueueItem {
protected:
    BackForwardItem(int howFar)
        : m_howFar(howFar)
    {
    }

private:
    virtual bool invoke() const;

    int m_howFar;
};

class BackItem : public BackForwardItem {
public:
    BackItem(unsigned howFar)
        : BackForwardItem(-howFar)
    {
    }
};

class ForwardItem : public BackForwardItem {
public:
    ForwardItem(unsigned howFar)
        : BackForwardItem(howFar)
    {
    }
};

#endif // !defined(WorkQueueItem_h)
