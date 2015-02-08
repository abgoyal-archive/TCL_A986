

#ifndef QWEBINSPECTOR_H
#define QWEBINSPECTOR_H

#include "qwebkitglobal.h"
#include "qwebpage.h"

#include "qwebview.h"

class QWebInspectorPrivate;

class QWEBKIT_EXPORT QWebInspector : public QWidget {
    Q_OBJECT
public:
    QWebInspector(QWidget* parent = 0);
    ~QWebInspector();

    void setPage(QWebPage* page);
    QWebPage* page() const;

    QSize sizeHint() const;
    bool event(QEvent*);

protected:
    void resizeEvent(QResizeEvent* event);
    void showEvent(QShowEvent* event);
    void hideEvent(QHideEvent* event);
    void closeEvent(QCloseEvent* event);

private:
    QWebInspectorPrivate* d;

    friend class QWebInspectorPrivate;
    friend class QWebPage;
    friend class QWebPagePrivate;
    friend class WebCore::InspectorClientQt;
};
#endif
