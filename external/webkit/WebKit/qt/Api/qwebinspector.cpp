

#include "config.h"
#include "qwebinspector.h"

#include "Element.h"
#include "InspectorController.h"
#include "qwebelement.h"
#include "qwebinspector_p.h"
#include "qwebpage_p.h"

#include <QResizeEvent>


QWebInspector::QWebInspector(QWidget* parent)
    : QWidget(parent)
    , d(new QWebInspectorPrivate(this))
{
}

QWebInspector::~QWebInspector()
{
    // Remove association principally to prevent deleting a child frontend
    setPage(0);
}

void QWebInspector::setPage(QWebPage* page)
{
    if (d->page) {
        // Break currentPage-->this
        d->page->d->setInspector(0);
    }
    if (page && page->d->inspector && page->d->inspector != this) {
        // Break newPage<->newPageCurrentInspector
        page->d->inspector->setPage(0);
    }

    d->page = page;

    if (page) {
        // Setup the reciprocal association
        page->d->setInspector(this);
    }
}

QWebPage* QWebInspector::page() const
{
    return d->page;
}

/*! \reimp */
QSize QWebInspector::sizeHint() const
{
    return QSize(450, 300);
}

/*! \reimp */
bool QWebInspector::event(QEvent* ev)
{
    return QWidget::event(ev);
}

/*! \reimp */
void QWebInspector::resizeEvent(QResizeEvent* event)
{
    d->adjustFrontendSize(event->size());
}

/*! \reimp */
void QWebInspector::showEvent(QShowEvent* event)
{
#if ENABLE(INSPECTOR)
    // Allows QWebInspector::show() to init the inspector.
    if (d->page)
        d->page->d->inspectorController()->show();
#endif
}

/*! \reimp */
void QWebInspector::hideEvent(QHideEvent* event)
{
}

/*! \reimp */
void QWebInspector::closeEvent(QCloseEvent* event)
{
#if ENABLE(INSPECTOR)
    if (d->page)
        d->page->d->inspectorController()->setWindowVisible(false);
#endif
}

/*! \internal */
void QWebInspectorPrivate::setFrontend(QWidget* newFrontend)
{
    if (frontend)
        frontend->setParent(0);

    frontend = newFrontend;

    if (frontend) {
        frontend->setParent(q);
        frontend->show();
        adjustFrontendSize(q->size());
    }
}

void QWebInspectorPrivate::adjustFrontendSize(const QSize& size)
{
    if (frontend)
        frontend->resize(size);
}

