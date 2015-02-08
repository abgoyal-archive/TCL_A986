

#ifndef LayoutTestControllerQt_h
#define LayoutTestControllerQt_h

#include <QBasicTimer>
#include <QObject>
#include <QSize>
#include <QString>
#include <QtDebug>
#include <QTimer>
#include <QTimerEvent>
#include <QVariant>

#include <qwebdatabase.h>
#include <qwebframe.h>
#include <qwebhistory.h>
#include <qwebpage.h>
#include <qwebsecurityorigin.h>

class QWebFrame;
namespace WebCore {
    class DumpRenderTree;
}
class LayoutTestController : public QObject {
    Q_OBJECT
    Q_PROPERTY(int webHistoryItemCount READ webHistoryItemCount)
    Q_PROPERTY(int workerThreadCount READ workerThreadCount)
    Q_PROPERTY(bool globalFlag READ globalFlag WRITE setGlobalFlag)
public:
    LayoutTestController(WebCore::DumpRenderTree* drt);

    bool shouldDumpAsText() const { return m_textDump; }
    bool shouldDumpBackForwardList() const { return m_dumpBackForwardList; }
    bool shouldDumpChildrenAsText() const { return m_dumpChildrenAsText; }
    bool shouldDumpDatabaseCallbacks() const { return m_dumpDatabaseCallbacks; }
    bool shouldDumpStatusCallbacks() const { return m_dumpStatusCallbacks; }
    bool shouldWaitUntilDone() const { return m_waitForDone; }
    bool shouldHandleErrorPages() const { return m_handleErrorPages; }
    bool canOpenWindows() const { return m_canOpenWindows; }
    bool shouldDumpTitleChanges() const { return m_dumpTitleChanges; }
    bool waitForPolicy() const { return m_waitForPolicy; }

    void reset();

protected:
    void timerEvent(QTimerEvent*);

signals:
    void done();

    void showPage();
    void hidePage();

public slots:
    void maybeDump(bool ok);
    void dumpAsText() { m_textDump = true; }
    void dumpChildFramesAsText() { m_dumpChildrenAsText = true; }
    void dumpDatabaseCallbacks() { m_dumpDatabaseCallbacks = true; }
    void dumpStatusCallbacks() { m_dumpStatusCallbacks = true; }
    void setCanOpenWindows() { m_canOpenWindows = true; }
    void waitUntilDone();
    QString counterValueForElementById(const QString& id);
    int webHistoryItemCount();
    void keepWebHistory();
    void notifyDone();
    void dumpBackForwardList() { m_dumpBackForwardList = true; }
    bool globalFlag() const { return m_globalFlag; }
    void setGlobalFlag(bool flag) { m_globalFlag = flag; }
    void handleErrorPages() { m_handleErrorPages = true; }
    void dumpEditingCallbacks();
    void dumpFrameLoadCallbacks();
    void dumpResourceLoadCallbacks();
    void queueBackNavigation(int howFarBackward);
    void queueForwardNavigation(int howFarForward);
    void queueLoad(const QString& url, const QString& target = QString());
    void queueReload();
    void queueLoadingScript(const QString& script);
    void queueNonLoadingScript(const QString& script);
    void provisionalLoad();
    void setCloseRemainingWindowsWhenComplete(bool = false) {}
    int windowCount();
    void display();
    void clearBackForwardList();
    QString pathToLocalResource(const QString& url);
    void dumpTitleChanges() { m_dumpTitleChanges = true; }
    QString encodeHostName(const QString& host);
    QString decodeHostName(const QString& host);
    void dumpSelectionRect() const {}
    void showWebInspector();
    void hideWebInspector();

    void setFrameSetFlatteningEnabled(bool enable);
    void setAllowUniversalAccessFromFileURLs(bool enable);
    void setJavaScriptProfilingEnabled(bool enable);
    void setFixedContentsSize(int width, int height);
    void setPrivateBrowsingEnabled(bool enable);
    void setPopupBlockingEnabled(bool enable);
    void setPOSIXLocale(const QString& locale);
    void resetLoadFinished() { m_loadFinished = false; }
    void setWindowIsKey(bool isKey);
    void setMainFrameIsFirstResponder(bool isFirst);
    void setXSSAuditorEnabled(bool enable);

    bool pauseAnimationAtTimeOnElementWithId(const QString& animationName, double time, const QString& elementId);
    bool pauseTransitionAtTimeOnElementWithId(const QString& propertyName, double time, const QString& elementId);
    bool sampleSVGAnimationForElementAtTime(const QString& animationId, double time, const QString& elementId);

    unsigned numberOfActiveAnimations() const;

    void whiteListAccessFromOrigin(const QString& sourceOrigin, const QString& destinationProtocol, const QString& destinationHost, bool allowDestinationSubdomains);

    void dispatchPendingLoadRequests();
    void disableImageLoading();

    void setDatabaseQuota(int size);
    void clearAllDatabases();

    void waitForPolicyDelegate();
    void overridePreference(const QString& name, const QVariant& value);
    void setUserStyleSheetLocation(const QString& url);
    void setUserStyleSheetEnabled(bool enabled);
    void setDomainRelaxationForbiddenForURLScheme(bool forbidden, const QString& scheme);
    int workerThreadCount();
    int pageNumberForElementById(const QString& id, float width = 0, float height = 0);

private slots:
    void processWork();

private:
    bool m_hasDumped;
    bool m_textDump;
    bool m_dumpBackForwardList;
    bool m_dumpChildrenAsText;
    bool m_canOpenWindows;
    bool m_waitForDone;
    bool m_dumpTitleChanges;
    bool m_dumpDatabaseCallbacks;
    bool m_dumpStatusCallbacks;
    bool m_waitForPolicy;
    bool m_handleErrorPages;
    bool m_loadFinished;
    bool m_globalFlag;

    QUrl m_userStyleSheetLocation;
    QBasicTimer m_timeoutTimer;
    QWebFrame* m_topLoadingFrame;
    WebCore::DumpRenderTree* m_drt;
    QWebHistory* m_webHistory;
};

#endif // LayoutTestControllerQt_h
