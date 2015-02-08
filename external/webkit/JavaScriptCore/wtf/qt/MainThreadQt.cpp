

#include "config.h"
#include "MainThread.h"

#include <QtCore/QObject>
#include <QtCore/QCoreApplication>


namespace WTF {

class MainThreadInvoker : public QObject {
    Q_OBJECT
public:
    MainThreadInvoker();

private Q_SLOTS:
    void dispatch();
};

MainThreadInvoker::MainThreadInvoker()
{
    moveToThread(QCoreApplication::instance()->thread());
}

void MainThreadInvoker::dispatch()
{
    dispatchFunctionsFromMainThread();
}

Q_GLOBAL_STATIC(MainThreadInvoker, webkit_main_thread_invoker)

void initializeMainThreadPlatform()
{
}

void scheduleDispatchFunctionsOnMainThread()
{
    QMetaObject::invokeMethod(webkit_main_thread_invoker(), "dispatch", Qt::QueuedConnection);
}

} // namespace WTF

#include "MainThreadQt.moc"
