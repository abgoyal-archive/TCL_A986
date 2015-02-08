

#ifndef QWEBVIEW_H
#define QWEBVIEW_H

#include "qwebkitglobal.h"
#include "qwebpage.h"
#include <QtGui/qwidget.h>
#include <QtGui/qicon.h>
#include <QtGui/qpainter.h>
#include <QtCore/qurl.h>
#include <QtNetwork/qnetworkaccessmanager.h>

QT_BEGIN_NAMESPACE
class QNetworkRequest;
class QPrinter;
QT_END_NAMESPACE

class QWebPage;
class QWebViewPrivate;
class QWebNetworkRequest;

class QWEBKIT_EXPORT QWebView : public QWidget {
    Q_OBJECT
    Q_PROPERTY(QString title READ title)
    Q_PROPERTY(QUrl url READ url WRITE setUrl)
    Q_PROPERTY(QIcon icon READ icon)
    Q_PROPERTY(QString selectedText READ selectedText)
    Q_PROPERTY(bool modified READ isModified)
    //Q_PROPERTY(Qt::TextInteractionFlags textInteractionFlags READ textInteractionFlags WRITE setTextInteractionFlags)
    Q_PROPERTY(qreal textSizeMultiplier READ textSizeMultiplier WRITE setTextSizeMultiplier DESIGNABLE false)
    Q_PROPERTY(qreal zoomFactor READ zoomFactor WRITE setZoomFactor)

    Q_PROPERTY(QPainter::RenderHints renderHints READ renderHints WRITE setRenderHints)
    Q_FLAGS(QPainter::RenderHints)
public:
    explicit QWebView(QWidget* parent = 0);
    virtual ~QWebView();

    QWebPage* page() const;
    void setPage(QWebPage* page);

    void load(const QUrl& url);
    void load(const QNetworkRequest& request,
              QNetworkAccessManager::Operation operation = QNetworkAccessManager::GetOperation,
              const QByteArray &body = QByteArray());
    void setHtml(const QString& html, const QUrl& baseUrl = QUrl());
    void setContent(const QByteArray& data, const QString& mimeType = QString(), const QUrl& baseUrl = QUrl());

    QWebHistory* history() const;
    QWebSettings* settings() const;

    QString title() const;
    void setUrl(const QUrl &url);
    QUrl url() const;
    QIcon icon() const;

    QString selectedText() const;

    QAction* pageAction(QWebPage::WebAction action) const;
    void triggerPageAction(QWebPage::WebAction action, bool checked = false);

    bool isModified() const;

    /*
    Qt::TextInteractionFlags textInteractionFlags() const;
    void setTextInteractionFlags(Qt::TextInteractionFlags flags);
    void setTextInteractionFlag(Qt::TextInteractionFlag flag);
    */

    QVariant inputMethodQuery(Qt::InputMethodQuery property) const;

    QSize sizeHint() const;

    qreal zoomFactor() const;
    void setZoomFactor(qreal factor);

    void setTextSizeMultiplier(qreal factor);
    qreal textSizeMultiplier() const;

    QPainter::RenderHints renderHints() const;
    void setRenderHints(QPainter::RenderHints hints);
    void setRenderHint(QPainter::RenderHint hint, bool enabled = true);

    bool findText(const QString& subString, QWebPage::FindFlags options = 0);

    virtual bool event(QEvent*);

public Q_SLOTS:
    void stop();
    void back();
    void forward();
    void reload();

    void print(QPrinter*) const;

Q_SIGNALS:
    void loadStarted();
    void loadProgress(int progress);
    void loadFinished(bool);
    void titleChanged(const QString& title);
    void statusBarMessage(const QString& text);
    void linkClicked(const QUrl&);
    void selectionChanged();
    void iconChanged();
    void urlChanged(const QUrl&);

protected:
    void resizeEvent(QResizeEvent*);
    void paintEvent(QPaintEvent*);

    virtual QWebView *createWindow(QWebPage::WebWindowType type);

    virtual void changeEvent(QEvent*);
    virtual void mouseMoveEvent(QMouseEvent*);
    virtual void mousePressEvent(QMouseEvent*);
    virtual void mouseDoubleClickEvent(QMouseEvent*);
    virtual void mouseReleaseEvent(QMouseEvent*);
#ifndef QT_NO_CONTEXTMENU
    virtual void contextMenuEvent(QContextMenuEvent*);
#endif
#ifndef QT_NO_WHEELEVENT
    virtual void wheelEvent(QWheelEvent*);
#endif
    virtual void keyPressEvent(QKeyEvent*);
    virtual void keyReleaseEvent(QKeyEvent*);
    virtual void dragEnterEvent(QDragEnterEvent*);
    virtual void dragLeaveEvent(QDragLeaveEvent*);
    virtual void dragMoveEvent(QDragMoveEvent*);
    virtual void dropEvent(QDropEvent*);
    virtual void focusInEvent(QFocusEvent*);
    virtual void focusOutEvent(QFocusEvent*);
    virtual void inputMethodEvent(QInputMethodEvent*);

    virtual bool focusNextPrevChild(bool next);

private:
    friend class QWebPage;
    QWebViewPrivate* d;
    Q_PRIVATE_SLOT(d, void _q_pageDestroyed())
};

#endif // QWEBVIEW_H
