

#include "config.h"
#include "qwebpage.h"
#include "qwebview.h"
#include "qwebframe.h"
#include "qwebpage_p.h"
#include "qwebframe_p.h"
#include "qwebhistory.h"
#include "qwebhistory_p.h"
#include "qwebinspector.h"
#include "qwebinspector_p.h"
#include "qwebsettings.h"
#include "qwebkitversion.h"

#include "Chrome.h"
#include "ContextMenuController.h"
#include "Frame.h"
#include "FrameTree.h"
#include "FrameLoader.h"
#include "FrameLoaderClientQt.h"
#include "FrameView.h"
#include "FormState.h"
#include "ApplicationCacheStorage.h"
#include "ChromeClientQt.h"
#include "ContextMenu.h"
#include "ContextMenuClientQt.h"
#include "DocumentLoader.h"
#include "DragClientQt.h"
#include "DragController.h"
#include "DragData.h"
#include "EditorClientQt.h"
#include "SecurityOrigin.h"
#include "Settings.h"
#include "Page.h"
#include "Pasteboard.h"
#include "FrameLoader.h"
#include "FrameLoadRequest.h"
#include "KURL.h"
#include "Logging.h"
#include "Image.h"
#include "InspectorClientQt.h"
#include "InspectorController.h"
#include "FocusController.h"
#include "Editor.h"
#include "Scrollbar.h"
#include "PlatformKeyboardEvent.h"
#include "PlatformWheelEvent.h"
#include "PluginDatabase.h"
#include "ProgressTracker.h"
#include "RefPtr.h"
#include "RenderTextControl.h"
#include "TextIterator.h"
#include "HashMap.h"
#include "HTMLFormElement.h"
#include "HTMLInputElement.h"
#include "HTMLNames.h"
#include "HitTestResult.h"
#include "WindowFeatures.h"
#include "LocalizedStrings.h"
#include "Cache.h"
#include "runtime/InitializeThreading.h"
#include "PageGroup.h"
#include "QWebPageClient.h"
#include "WorkerThread.h"

#include <QApplication>
#include <QBasicTimer>
#include <QBitArray>
#include <QDebug>
#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QFileDialog>
#include <QHttpRequestHeader>
#include <QInputDialog>
#include <QLocale>
#include <QMessageBox>
#include <QNetworkProxy>
#include <QUndoStack>
#include <QUrl>
#include <QPainter>
#include <QClipboard>
#include <QSslSocket>
#include <QStyle>
#include <QSysInfo>
#include <QTextCharFormat>
#include <QTextDocument>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#if defined(Q_WS_X11)
#include <QX11Info>
#endif

#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
#include <QTouchEvent>
#include "PlatformTouchEvent.h"
#endif

using namespace WebCore;

void QWEBKIT_EXPORT qt_drt_overwritePluginDirectories()
{
    PluginDatabase* db = PluginDatabase::installedPlugins(/* populate */ false);

    Vector<String> paths;
    String qtPath(qgetenv("QTWEBKIT_PLUGIN_PATH").data());
    qtPath.split(UChar(':'), /* allowEmptyEntries */ false, paths);

    db->setPluginDirectories(paths);
    db->refresh();
}

int QWEBKIT_EXPORT qt_drt_workerThreadCount()
{
#if ENABLE(WORKERS)
    return WebCore::WorkerThread::workerThreadCount();
#else
    return 0;
#endif
}

bool QWebPagePrivate::drtRun = false;
void QWEBKIT_EXPORT qt_drt_run(bool b)
{
    QWebPagePrivate::drtRun = b;
}

void QWEBKIT_EXPORT qt_drt_setFrameSetFlatteningEnabled(QWebPage* page, bool enabled)
{
    QWebPagePrivate::core(page)->settings()->setFrameSetFlatteningEnabled(enabled);
}

void QWEBKIT_EXPORT qt_webpage_setGroupName(QWebPage* page, const QString& groupName)
{
    page->handle()->page->setGroupName(groupName);
}

QString QWEBKIT_EXPORT qt_webpage_groupName(QWebPage* page)
{
    return page->handle()->page->groupName();
}

class QWebPageWidgetClient : public QWebPageClient {
public:
    QWebPageWidgetClient(QWidget* view)
        : view(view)
    {
        Q_ASSERT(view);
    }

    virtual void scroll(int dx, int dy, const QRect&);
    virtual void update(const QRect& dirtyRect);
    virtual void setInputMethodEnabled(bool enable);
    virtual bool inputMethodEnabled() const;
#if QT_VERSION >= 0x040600
    virtual void setInputMethodHint(Qt::InputMethodHint hint, bool enable);
#endif

#ifndef QT_NO_CURSOR
    virtual QCursor cursor() const;
    virtual void updateCursor(const QCursor& cursor);
#endif

    virtual QPalette palette() const;
    virtual int screenNumber() const;
    virtual QWidget* ownerWidget() const;

    virtual QObject* pluginParent() const;

    virtual QStyle* style() const;

    QWidget* view;
};

void QWebPageWidgetClient::scroll(int dx, int dy, const QRect& rectToScroll)
{
    view->scroll(qreal(dx), qreal(dy), rectToScroll);
}

void QWebPageWidgetClient::update(const QRect & dirtyRect)
{
    view->update(dirtyRect);
}

void QWebPageWidgetClient::setInputMethodEnabled(bool enable)
{
    view->setAttribute(Qt::WA_InputMethodEnabled, enable);
}

bool QWebPageWidgetClient::inputMethodEnabled() const
{
    return view->testAttribute(Qt::WA_InputMethodEnabled);
}

#if QT_VERSION >= 0x040600
void QWebPageWidgetClient::setInputMethodHint(Qt::InputMethodHint hint, bool enable)
{
    if (enable)
        view->setInputMethodHints(view->inputMethodHints() | hint);
    else
        view->setInputMethodHints(view->inputMethodHints() & ~hint);
}
#endif
#ifndef QT_NO_CURSOR
QCursor QWebPageWidgetClient::cursor() const
{
    return view->cursor();
}

void QWebPageWidgetClient::updateCursor(const QCursor& cursor)
{
    view->setCursor(cursor);
}
#endif

QPalette QWebPageWidgetClient::palette() const
{
    return view->palette();
}

int QWebPageWidgetClient::screenNumber() const
{
#if defined(Q_WS_X11)
    if (view)
        return view->x11Info().screen();
#endif

    return 0;
}

QWidget* QWebPageWidgetClient::ownerWidget() const
{
    return view;
}

QObject* QWebPageWidgetClient::pluginParent() const
{
    return view;
}

QStyle* QWebPageWidgetClient::style() const
{
    return view->style();
}

// Lookup table mapping QWebPage::WebActions to the associated Editor commands
static const char* editorCommandWebActions[] =
{
    0, // OpenLink,

    0, // OpenLinkInNewWindow,
    0, // OpenFrameInNewWindow,

    0, // DownloadLinkToDisk,
    0, // CopyLinkToClipboard,

    0, // OpenImageInNewWindow,
    0, // DownloadImageToDisk,
    0, // CopyImageToClipboard,

    0, // Back,
    0, // Forward,
    0, // Stop,
    0, // Reload,

    "Cut", // Cut,
    "Copy", // Copy,
    "Paste", // Paste,

    "Undo", // Undo,
    "Redo", // Redo,
    "MoveForward", // MoveToNextChar,
    "MoveBackward", // MoveToPreviousChar,
    "MoveWordForward", // MoveToNextWord,
    "MoveWordBackward", // MoveToPreviousWord,
    "MoveDown", // MoveToNextLine,
    "MoveUp", // MoveToPreviousLine,
    "MoveToBeginningOfLine", // MoveToStartOfLine,
    "MoveToEndOfLine", // MoveToEndOfLine,
    "MoveToBeginningOfParagraph", // MoveToStartOfBlock,
    "MoveToEndOfParagraph", // MoveToEndOfBlock,
    "MoveToBeginningOfDocument", // MoveToStartOfDocument,
    "MoveToEndOfDocument", // MoveToEndOfDocument,
    "MoveForwardAndModifySelection", // SelectNextChar,
    "MoveBackwardAndModifySelection", // SelectPreviousChar,
    "MoveWordForwardAndModifySelection", // SelectNextWord,
    "MoveWordBackwardAndModifySelection", // SelectPreviousWord,
    "MoveDownAndModifySelection", // SelectNextLine,
    "MoveUpAndModifySelection", // SelectPreviousLine,
    "MoveToBeginningOfLineAndModifySelection", // SelectStartOfLine,
    "MoveToEndOfLineAndModifySelection", // SelectEndOfLine,
    "MoveToBeginningOfParagraphAndModifySelection", // SelectStartOfBlock,
    "MoveToEndOfParagraphAndModifySelection", // SelectEndOfBlock,
    "MoveToBeginningOfDocumentAndModifySelection", //SelectStartOfDocument,
    "MoveToEndOfDocumentAndModifySelection", // SelectEndOfDocument,
    "DeleteWordBackward", // DeleteStartOfWord,
    "DeleteWordForward", // DeleteEndOfWord,

    0, // SetTextDirectionDefault,
    0, // SetTextDirectionLeftToRight,
    0, // SetTextDirectionRightToLeft,

    "ToggleBold", // ToggleBold,
    "ToggleItalic", // ToggleItalic,
    "ToggleUnderline", // ToggleUnderline,

    0, // InspectElement,

    "InsertNewline", // InsertParagraphSeparator
    "InsertLineBreak", // InsertLineSeparator

    "SelectAll", // SelectAll
    0, // ReloadAndBypassCache,

    "PasteAndMatchStyle", // PasteAndMatchStyle
    "RemoveFormat", // RemoveFormat
    "Strikethrough", // ToggleStrikethrough,
    "Subscript", // ToggleSubscript
    "Superscript", // ToggleSuperscript
    "InsertUnorderedList", // InsertUnorderedList
    "InsertOrderedList", // InsertOrderedList
    "Indent", // Indent
    "Outdent", // Outdent,

    "AlignCenter", // AlignCenter,
    "AlignJustified", // AlignJustified,
    "AlignLeft", // AlignLeft,
    "AlignRight", // AlignRight,

    0 // WebActionCount
};

// Lookup the appropriate editor command to use for WebAction \a action
const char* QWebPagePrivate::editorCommandForWebActions(QWebPage::WebAction action)
{
    if ((action > QWebPage::NoWebAction) && (action < int(sizeof(editorCommandWebActions) / sizeof(const char*))))
        return editorCommandWebActions[action];
    return 0;
}

// If you change this make sure to also adjust the docs for QWebPage::userAgentForUrl
#define WEBKIT_VERSION "527+"

static inline DragOperation dropActionToDragOp(Qt::DropActions actions)
{
    unsigned result = 0;
    if (actions & Qt::CopyAction)
        result |= DragOperationCopy;
    // DragOperationgeneric represents InternetExplorer's equivalent of Move operation,
    // hence it should be considered as "move"
    if (actions & Qt::MoveAction)
        result |= (DragOperationMove | DragOperationGeneric);
    if (actions & Qt::LinkAction)
        result |= DragOperationLink;
    return (DragOperation)result;
}

static inline Qt::DropAction dragOpToDropAction(unsigned actions)
{
    Qt::DropAction result = Qt::IgnoreAction;
    if (actions & DragOperationCopy)
        result = Qt::CopyAction;
    else if (actions & DragOperationMove)
        result = Qt::MoveAction;
    // DragOperationgeneric represents InternetExplorer's equivalent of Move operation,
    // hence it should be considered as "move"
    else if (actions & DragOperationGeneric)
        result = Qt::MoveAction;
    else if (actions & DragOperationLink)
        result = Qt::LinkAction;
    return result;
}

QWebPagePrivate::QWebPagePrivate(QWebPage *qq)
    : q(qq)
    , client(0)
#if QT_VERSION < 0x040600
    , view(0)
#endif
    , clickCausedFocus(false)
    , viewportSize(QSize(0, 0))
    , inspectorFrontend(0)
    , inspector(0)
    , inspectorIsInternalOnly(false)
{
    WebCore::InitializeLoggingChannelsIfNecessary();
    JSC::initializeThreading();
    WebCore::SecurityOrigin::setLocalLoadPolicy(WebCore::SecurityOrigin::AllowLocalLoadsForLocalAndSubstituteData);

    chromeClient = new ChromeClientQt(q);
    contextMenuClient = new ContextMenuClientQt();
    editorClient = new EditorClientQt(q);
    page = new Page(chromeClient, contextMenuClient, editorClient,
                    new DragClientQt(q), new InspectorClientQt(q), 0, 0);

    settings = new QWebSettings(page->settings());

#ifndef QT_NO_UNDOSTACK
    undoStack = 0;
#endif
    mainFrame = 0;
    networkManager = 0;
    pluginFactory = 0;
    insideOpenCall = false;
    forwardUnsupportedContent = false;
    editable = false;
    useFixedLayout = false;
    linkPolicy = QWebPage::DontDelegateLinks;
#ifndef QT_NO_CONTEXTMENU
    currentContextMenu = 0;
#endif

    history.d = new QWebHistoryPrivate(page->backForwardList());
    memset(actions, 0, sizeof(actions));

    PageGroup::setShouldTrackVisitedLinks(true);
}

QWebPagePrivate::~QWebPagePrivate()
{
#ifndef QT_NO_CONTEXTMENU
    delete currentContextMenu;
#endif
#ifndef QT_NO_UNDOSTACK
    delete undoStack;
#endif
    delete settings;
    delete page;
}

WebCore::Page* QWebPagePrivate::core(QWebPage* page)
{
    return page->d->page;
}

bool QWebPagePrivate::acceptNavigationRequest(QWebFrame *frame, const QNetworkRequest &request, QWebPage::NavigationType type)
{
    if (insideOpenCall
        && frame == mainFrame)
        return true;
    return q->acceptNavigationRequest(frame, request, type);
}

void QWebPagePrivate::createMainFrame()
{
    if (!mainFrame) {
        QWebFrameData frameData(page);
        mainFrame = new QWebFrame(q, &frameData);

        emit q->frameCreated(mainFrame);
    }
}

static QWebPage::WebAction webActionForContextMenuAction(WebCore::ContextMenuAction action)
{
    switch (action) {
        case WebCore::ContextMenuItemTagOpenLink: return QWebPage::OpenLink;
        case WebCore::ContextMenuItemTagOpenLinkInNewWindow: return QWebPage::OpenLinkInNewWindow;
        case WebCore::ContextMenuItemTagDownloadLinkToDisk: return QWebPage::DownloadLinkToDisk;
        case WebCore::ContextMenuItemTagCopyLinkToClipboard: return QWebPage::CopyLinkToClipboard;
        case WebCore::ContextMenuItemTagOpenImageInNewWindow: return QWebPage::OpenImageInNewWindow;
        case WebCore::ContextMenuItemTagDownloadImageToDisk: return QWebPage::DownloadImageToDisk;
        case WebCore::ContextMenuItemTagCopyImageToClipboard: return QWebPage::CopyImageToClipboard;
        case WebCore::ContextMenuItemTagOpenFrameInNewWindow: return QWebPage::OpenFrameInNewWindow;
        case WebCore::ContextMenuItemTagCopy: return QWebPage::Copy;
        case WebCore::ContextMenuItemTagGoBack: return QWebPage::Back;
        case WebCore::ContextMenuItemTagGoForward: return QWebPage::Forward;
        case WebCore::ContextMenuItemTagStop: return QWebPage::Stop;
        case WebCore::ContextMenuItemTagReload: return QWebPage::Reload;
        case WebCore::ContextMenuItemTagCut: return QWebPage::Cut;
        case WebCore::ContextMenuItemTagPaste: return QWebPage::Paste;
        case WebCore::ContextMenuItemTagDefaultDirection: return QWebPage::SetTextDirectionDefault;
        case WebCore::ContextMenuItemTagLeftToRight: return QWebPage::SetTextDirectionLeftToRight;
        case WebCore::ContextMenuItemTagRightToLeft: return QWebPage::SetTextDirectionRightToLeft;
        case WebCore::ContextMenuItemTagBold: return QWebPage::ToggleBold;
        case WebCore::ContextMenuItemTagItalic: return QWebPage::ToggleItalic;
        case WebCore::ContextMenuItemTagUnderline: return QWebPage::ToggleUnderline;
#if ENABLE(INSPECTOR)
        case WebCore::ContextMenuItemTagInspectElement: return QWebPage::InspectElement;
#endif
        default: break;
    }
    return QWebPage::NoWebAction;
}

#ifndef QT_NO_CONTEXTMENU
QMenu *QWebPagePrivate::createContextMenu(const WebCore::ContextMenu *webcoreMenu,
        const QList<WebCore::ContextMenuItem> *items, QBitArray *visitedWebActions)
{
    QMenu* menu = new QMenu(q->view());
    for (int i = 0; i < items->count(); ++i) {
        const ContextMenuItem &item = items->at(i);
        switch (item.type()) {
            case WebCore::CheckableActionType: /* fall through */
            case WebCore::ActionType: {
                QWebPage::WebAction action = webActionForContextMenuAction(item.action());
                QAction *a = q->action(action);
                if (a) {
                    ContextMenuItem it(item);
                    webcoreMenu->checkOrEnableIfNeeded(it);
                    PlatformMenuItemDescription desc = it.releasePlatformDescription();
                    a->setEnabled(desc.enabled);
                    a->setChecked(desc.checked);
                    a->setCheckable(item.type() == WebCore::CheckableActionType);

                    menu->addAction(a);
                    visitedWebActions->setBit(action);
                }
                break;
            }
            case WebCore::SeparatorType:
                menu->addSeparator();
                break;
            case WebCore::SubmenuType: {
                QMenu *subMenu = createContextMenu(webcoreMenu, item.platformSubMenu(), visitedWebActions);

                bool anyEnabledAction = false;

                QList<QAction *> actions = subMenu->actions();
                for (int i = 0; i < actions.count(); ++i) {
                    if (actions.at(i)->isVisible())
                        anyEnabledAction |= actions.at(i)->isEnabled();
                }

                // don't show sub-menus with just disabled actions
                if (anyEnabledAction) {
                    subMenu->setTitle(item.title());
                    menu->addAction(subMenu->menuAction());
                } else
                    delete subMenu;
                break;
            }
        }
    }
    return menu;
}
#endif // QT_NO_CONTEXTMENU

void QWebPagePrivate::_q_webActionTriggered(bool checked)
{
    QAction *a = qobject_cast<QAction *>(q->sender());
    if (!a)
        return;
    QWebPage::WebAction action = static_cast<QWebPage::WebAction>(a->data().toInt());
    q->triggerAction(action, checked);
}

void QWebPagePrivate::_q_cleanupLeakMessages()
{
#ifndef NDEBUG
    // Need this to make leak messages accurate.
    cache()->setCapacities(0, 0, 0);
#endif
}

void QWebPagePrivate::updateAction(QWebPage::WebAction action)
{
    QAction *a = actions[action];
    if (!a || !mainFrame)
        return;

    WebCore::FrameLoader *loader = mainFrame->d->frame->loader();
    WebCore::Editor *editor = page->focusController()->focusedOrMainFrame()->editor();

    bool enabled = a->isEnabled();
    bool checked = a->isChecked();

    switch (action) {
        case QWebPage::Back:
            enabled = page->canGoBackOrForward(-1);
            break;
        case QWebPage::Forward:
            enabled = page->canGoBackOrForward(1);
            break;
        case QWebPage::Stop:
            enabled = loader->isLoading();
            break;
        case QWebPage::Reload:
        case QWebPage::ReloadAndBypassCache:
            enabled = !loader->isLoading();
            break;
#ifndef QT_NO_UNDOSTACK
        case QWebPage::Undo:
        case QWebPage::Redo:
            // those two are handled by QUndoStack
            break;
#endif // QT_NO_UNDOSTACK
        case QWebPage::SelectAll: // editor command is always enabled
            break;
        case QWebPage::SetTextDirectionDefault:
        case QWebPage::SetTextDirectionLeftToRight:
        case QWebPage::SetTextDirectionRightToLeft:
            enabled = editor->canEdit();
            checked = false;
            break;
        default: {
            // see if it's an editor command
            const char* commandName = editorCommandForWebActions(action);

            // if it's an editor command, let it's logic determine state
            if (commandName) {
                Editor::Command command = editor->command(commandName);
                enabled = command.isEnabled();
                if (enabled)
                    checked = command.state() != FalseTriState;
                else
                    checked = false;
            }
            break;
        }
    }

    a->setEnabled(enabled);

    if (a->isCheckable())
        a->setChecked(checked);
}

void QWebPagePrivate::updateNavigationActions()
{
    updateAction(QWebPage::Back);
    updateAction(QWebPage::Forward);
    updateAction(QWebPage::Stop);
    updateAction(QWebPage::Reload);
    updateAction(QWebPage::ReloadAndBypassCache);
}

void QWebPagePrivate::updateEditorActions()
{
    updateAction(QWebPage::Cut);
    updateAction(QWebPage::Copy);
    updateAction(QWebPage::Paste);
    updateAction(QWebPage::MoveToNextChar);
    updateAction(QWebPage::MoveToPreviousChar);
    updateAction(QWebPage::MoveToNextWord);
    updateAction(QWebPage::MoveToPreviousWord);
    updateAction(QWebPage::MoveToNextLine);
    updateAction(QWebPage::MoveToPreviousLine);
    updateAction(QWebPage::MoveToStartOfLine);
    updateAction(QWebPage::MoveToEndOfLine);
    updateAction(QWebPage::MoveToStartOfBlock);
    updateAction(QWebPage::MoveToEndOfBlock);
    updateAction(QWebPage::MoveToStartOfDocument);
    updateAction(QWebPage::MoveToEndOfDocument);
    updateAction(QWebPage::SelectNextChar);
    updateAction(QWebPage::SelectPreviousChar);
    updateAction(QWebPage::SelectNextWord);
    updateAction(QWebPage::SelectPreviousWord);
    updateAction(QWebPage::SelectNextLine);
    updateAction(QWebPage::SelectPreviousLine);
    updateAction(QWebPage::SelectStartOfLine);
    updateAction(QWebPage::SelectEndOfLine);
    updateAction(QWebPage::SelectStartOfBlock);
    updateAction(QWebPage::SelectEndOfBlock);
    updateAction(QWebPage::SelectStartOfDocument);
    updateAction(QWebPage::SelectEndOfDocument);
    updateAction(QWebPage::DeleteStartOfWord);
    updateAction(QWebPage::DeleteEndOfWord);
    updateAction(QWebPage::SetTextDirectionDefault);
    updateAction(QWebPage::SetTextDirectionLeftToRight);
    updateAction(QWebPage::SetTextDirectionRightToLeft);
    updateAction(QWebPage::ToggleBold);
    updateAction(QWebPage::ToggleItalic);
    updateAction(QWebPage::ToggleUnderline);
    updateAction(QWebPage::InsertParagraphSeparator);
    updateAction(QWebPage::InsertLineSeparator);
    updateAction(QWebPage::PasteAndMatchStyle);
    updateAction(QWebPage::RemoveFormat);
    updateAction(QWebPage::ToggleStrikethrough);
    updateAction(QWebPage::ToggleSubscript);
    updateAction(QWebPage::ToggleSuperscript);
    updateAction(QWebPage::InsertUnorderedList);
    updateAction(QWebPage::InsertOrderedList);
    updateAction(QWebPage::Indent);
    updateAction(QWebPage::Outdent);
    updateAction(QWebPage::AlignCenter);
    updateAction(QWebPage::AlignJustified);
    updateAction(QWebPage::AlignLeft);
    updateAction(QWebPage::AlignRight);
}

void QWebPagePrivate::timerEvent(QTimerEvent *ev)
{
    int timerId = ev->timerId();
    if (timerId == tripleClickTimer.timerId())
        tripleClickTimer.stop();
    else
        q->QObject::timerEvent(ev);
}

void QWebPagePrivate::mouseMoveEvent(QGraphicsSceneMouseEvent* ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    bool accepted = frame->eventHandler()->mouseMoved(PlatformMouseEvent(ev, 0));
    ev->setAccepted(accepted);
}

void QWebPagePrivate::mouseMoveEvent(QMouseEvent *ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    bool accepted = frame->eventHandler()->mouseMoved(PlatformMouseEvent(ev, 0));
    ev->setAccepted(accepted);
}

void QWebPagePrivate::mousePressEvent(QGraphicsSceneMouseEvent* ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    if (tripleClickTimer.isActive()
            && (ev->pos().toPoint() - tripleClick).manhattanLength()
                < QApplication::startDragDistance()) {
        mouseTripleClickEvent(ev);
        return;
    }

    bool accepted = false;
    PlatformMouseEvent mev(ev, 1);
    // ignore the event if we can't map Qt's mouse buttons to WebCore::MouseButton
    if (mev.button() != NoButton)
        accepted = frame->eventHandler()->handleMousePressEvent(mev);
    ev->setAccepted(accepted);
}

void QWebPagePrivate::mousePressEvent(QMouseEvent *ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    RefPtr<WebCore::Node> oldNode;
    if (page->focusController()->focusedFrame()
        && page->focusController()->focusedFrame()->document())
        oldNode = page->focusController()->focusedFrame()->document()->focusedNode();

    if (tripleClickTimer.isActive()
            && (ev->pos() - tripleClick).manhattanLength()
                < QApplication::startDragDistance()) {
        mouseTripleClickEvent(ev);
        return;
    }

    bool accepted = false;
    PlatformMouseEvent mev(ev, 1);
    // ignore the event if we can't map Qt's mouse buttons to WebCore::MouseButton
    if (mev.button() != NoButton)
        accepted = frame->eventHandler()->handleMousePressEvent(mev);
    ev->setAccepted(accepted);

    RefPtr<WebCore::Node> newNode;
    if (page->focusController()->focusedFrame()
        && page->focusController()->focusedFrame()->document())
        newNode = page->focusController()->focusedFrame()->document()->focusedNode();

    if (newNode && oldNode != newNode)
        clickCausedFocus = true;
}

void QWebPagePrivate::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    bool accepted = false;
    PlatformMouseEvent mev(ev, 2);
    // ignore the event if we can't map Qt's mouse buttons to WebCore::MouseButton
    if (mev.button() != NoButton)
        accepted = frame->eventHandler()->handleMousePressEvent(mev);
    ev->setAccepted(accepted);

    tripleClickTimer.start(QApplication::doubleClickInterval(), q);
    tripleClick = ev->pos().toPoint();
}

void QWebPagePrivate::mouseDoubleClickEvent(QMouseEvent *ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    bool accepted = false;
    PlatformMouseEvent mev(ev, 2);
    // ignore the event if we can't map Qt's mouse buttons to WebCore::MouseButton
    if (mev.button() != NoButton)
        accepted = frame->eventHandler()->handleMousePressEvent(mev);
    ev->setAccepted(accepted);

    tripleClickTimer.start(QApplication::doubleClickInterval(), q);
    tripleClick = ev->pos();
}

void QWebPagePrivate::mouseTripleClickEvent(QGraphicsSceneMouseEvent *ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    bool accepted = false;
    PlatformMouseEvent mev(ev, 3);
    // ignore the event if we can't map Qt's mouse buttons to WebCore::MouseButton
    if (mev.button() != NoButton)
        accepted = frame->eventHandler()->handleMousePressEvent(mev);
    ev->setAccepted(accepted);
}

void QWebPagePrivate::mouseTripleClickEvent(QMouseEvent *ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    bool accepted = false;
    PlatformMouseEvent mev(ev, 3);
    // ignore the event if we can't map Qt's mouse buttons to WebCore::MouseButton
    if (mev.button() != NoButton)
        accepted = frame->eventHandler()->handleMousePressEvent(mev);
    ev->setAccepted(accepted);
}

void QWebPagePrivate::handleClipboard(QEvent* ev, Qt::MouseButton button)
{
#ifndef QT_NO_CLIPBOARD
    if (QApplication::clipboard()->supportsSelection()) {
        bool oldSelectionMode = Pasteboard::generalPasteboard()->isSelectionMode();
        Pasteboard::generalPasteboard()->setSelectionMode(true);
        WebCore::Frame* focusFrame = page->focusController()->focusedOrMainFrame();
        if (button == Qt::LeftButton) {
            if (focusFrame && (focusFrame->editor()->canCopy() || focusFrame->editor()->canDHTMLCopy())) {
                focusFrame->editor()->copy();
                ev->setAccepted(true);
            }
        } else if (button == Qt::MidButton) {
            if (focusFrame && (focusFrame->editor()->canPaste() || focusFrame->editor()->canDHTMLPaste())) {
                focusFrame->editor()->paste();
                ev->setAccepted(true);
            }
        }
        Pasteboard::generalPasteboard()->setSelectionMode(oldSelectionMode);
    }
#endif
}

void QWebPagePrivate::mouseReleaseEvent(QGraphicsSceneMouseEvent* ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    bool accepted = false;
    PlatformMouseEvent mev(ev, 0);
    // ignore the event if we can't map Qt's mouse buttons to WebCore::MouseButton
    if (mev.button() != NoButton)
        accepted = frame->eventHandler()->handleMouseReleaseEvent(mev);
    ev->setAccepted(accepted);

    handleClipboard(ev, ev->button());
    handleSoftwareInputPanel(ev->button());
}

void QWebPagePrivate::handleSoftwareInputPanel(Qt::MouseButton button)
{
#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
    Frame* frame = page->focusController()->focusedFrame();
    if (!frame)
        return;

    if (client && client->inputMethodEnabled()
        && frame->document()->focusedNode()
        && button == Qt::LeftButton && qApp->autoSipEnabled()) {
        QStyle::RequestSoftwareInputPanel behavior = QStyle::RequestSoftwareInputPanel(
            client->ownerWidget()->style()->styleHint(QStyle::SH_RequestSoftwareInputPanel));
        if (!clickCausedFocus || behavior == QStyle::RSIP_OnMouseClick) {
            QEvent event(QEvent::RequestSoftwareInputPanel);
            QApplication::sendEvent(client->ownerWidget(), &event);
        }
    }

    clickCausedFocus = false;
#endif
}

void QWebPagePrivate::mouseReleaseEvent(QMouseEvent *ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    bool accepted = false;
    PlatformMouseEvent mev(ev, 0);
    // ignore the event if we can't map Qt's mouse buttons to WebCore::MouseButton
    if (mev.button() != NoButton)
        accepted = frame->eventHandler()->handleMouseReleaseEvent(mev);
    ev->setAccepted(accepted);

    handleClipboard(ev, ev->button());
    handleSoftwareInputPanel(ev->button());
}

#ifndef QT_NO_CONTEXTMENU
void QWebPagePrivate::contextMenuEvent(const QPoint& globalPos)
{
    QMenu *menu = q->createStandardContextMenu();
    if (menu) {
        menu->exec(globalPos);
        delete menu;
    }
}
#endif // QT_NO_CONTEXTMENU

QMenu *QWebPage::createStandardContextMenu()
{
#ifndef QT_NO_CONTEXTMENU
    QMenu *menu = d->currentContextMenu;
    d->currentContextMenu = 0;
    return menu;
#else
    return 0;
#endif
}

#ifndef QT_NO_WHEELEVENT
void QWebPagePrivate::wheelEvent(QGraphicsSceneWheelEvent* ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    WebCore::PlatformWheelEvent pev(ev);
    bool accepted = frame->eventHandler()->handleWheelEvent(pev);
    ev->setAccepted(accepted);
}

void QWebPagePrivate::wheelEvent(QWheelEvent *ev)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    WebCore::PlatformWheelEvent pev(ev);
    bool accepted = frame->eventHandler()->handleWheelEvent(pev);
    ev->setAccepted(accepted);
}
#endif // QT_NO_WHEELEVENT

#ifndef QT_NO_SHORTCUT
QWebPage::WebAction QWebPagePrivate::editorActionForKeyEvent(QKeyEvent* event)
{
    static struct {
        QKeySequence::StandardKey standardKey;
        QWebPage::WebAction action;
    } editorActions[] = {
        { QKeySequence::Cut, QWebPage::Cut },
        { QKeySequence::Copy, QWebPage::Copy },
        { QKeySequence::Paste, QWebPage::Paste },
        { QKeySequence::Undo, QWebPage::Undo },
        { QKeySequence::Redo, QWebPage::Redo },
        { QKeySequence::MoveToNextChar, QWebPage::MoveToNextChar },
        { QKeySequence::MoveToPreviousChar, QWebPage::MoveToPreviousChar },
        { QKeySequence::MoveToNextWord, QWebPage::MoveToNextWord },
        { QKeySequence::MoveToPreviousWord, QWebPage::MoveToPreviousWord },
        { QKeySequence::MoveToNextLine, QWebPage::MoveToNextLine },
        { QKeySequence::MoveToPreviousLine, QWebPage::MoveToPreviousLine },
        { QKeySequence::MoveToStartOfLine, QWebPage::MoveToStartOfLine },
        { QKeySequence::MoveToEndOfLine, QWebPage::MoveToEndOfLine },
        { QKeySequence::MoveToStartOfBlock, QWebPage::MoveToStartOfBlock },
        { QKeySequence::MoveToEndOfBlock, QWebPage::MoveToEndOfBlock },
        { QKeySequence::MoveToStartOfDocument, QWebPage::MoveToStartOfDocument },
        { QKeySequence::MoveToEndOfDocument, QWebPage::MoveToEndOfDocument },
        { QKeySequence::SelectNextChar, QWebPage::SelectNextChar },
        { QKeySequence::SelectPreviousChar, QWebPage::SelectPreviousChar },
        { QKeySequence::SelectNextWord, QWebPage::SelectNextWord },
        { QKeySequence::SelectPreviousWord, QWebPage::SelectPreviousWord },
        { QKeySequence::SelectNextLine, QWebPage::SelectNextLine },
        { QKeySequence::SelectPreviousLine, QWebPage::SelectPreviousLine },
        { QKeySequence::SelectStartOfLine, QWebPage::SelectStartOfLine },
        { QKeySequence::SelectEndOfLine, QWebPage::SelectEndOfLine },
        { QKeySequence::SelectStartOfBlock, QWebPage::SelectStartOfBlock },
        { QKeySequence::SelectEndOfBlock,  QWebPage::SelectEndOfBlock },
        { QKeySequence::SelectStartOfDocument, QWebPage::SelectStartOfDocument },
        { QKeySequence::SelectEndOfDocument, QWebPage::SelectEndOfDocument },
        { QKeySequence::DeleteStartOfWord, QWebPage::DeleteStartOfWord },
        { QKeySequence::DeleteEndOfWord, QWebPage::DeleteEndOfWord },
#if QT_VERSION >= 0x040500
        { QKeySequence::InsertParagraphSeparator, QWebPage::InsertParagraphSeparator },
        { QKeySequence::InsertLineSeparator, QWebPage::InsertLineSeparator },
#endif
        { QKeySequence::SelectAll, QWebPage::SelectAll },
        { QKeySequence::UnknownKey, QWebPage::NoWebAction }
    };

    for (int i = 0; editorActions[i].standardKey != QKeySequence::UnknownKey; ++i)
        if (event == editorActions[i].standardKey)
            return editorActions[i].action;

    return QWebPage::NoWebAction;
}
#endif // QT_NO_SHORTCUT

void QWebPagePrivate::keyPressEvent(QKeyEvent *ev)
{
    bool handled = false;
    WebCore::Frame* frame = page->focusController()->focusedOrMainFrame();
    // we forward the key event to WebCore first to handle potential DOM
    // defined event handlers and later on end up in EditorClientQt::handleKeyboardEvent
    // to trigger editor commands via triggerAction().
    if (!handled)
        handled = frame->eventHandler()->keyEvent(ev);
    if (!handled) {
        handled = true;
        QFont defaultFont;
        if (q->view())
            defaultFont = q->view()->font();
        QFontMetrics fm(defaultFont);
        if (!handleScrolling(ev, frame)) {
            switch (ev->key()) {
            case Qt::Key_Back:
                q->triggerAction(QWebPage::Back);
                break;
            case Qt::Key_Forward:
                q->triggerAction(QWebPage::Forward);
                break;
            case Qt::Key_Stop:
                q->triggerAction(QWebPage::Stop);
                break;
            case Qt::Key_Refresh:
                q->triggerAction(QWebPage::Reload);
                break;
            case Qt::Key_Backspace:
                if (ev->modifiers() == Qt::ShiftModifier)
                    q->triggerAction(QWebPage::Forward);
                else
                    q->triggerAction(QWebPage::Back);
                break;
            default:
                handled = false;
                break;
            }
        }
    }

    ev->setAccepted(handled);
}

void QWebPagePrivate::keyReleaseEvent(QKeyEvent *ev)
{
    if (ev->isAutoRepeat()) {
        ev->setAccepted(true);
        return;
    }

    WebCore::Frame* frame = page->focusController()->focusedOrMainFrame();
    bool handled = frame->eventHandler()->keyEvent(ev);
    ev->setAccepted(handled);
}

void QWebPagePrivate::focusInEvent(QFocusEvent*)
{
    FocusController *focusController = page->focusController();
    focusController->setActive(true);
    focusController->setFocused(true);
    if (!focusController->focusedFrame())
        focusController->setFocusedFrame(QWebFramePrivate::core(mainFrame));
}

void QWebPagePrivate::focusOutEvent(QFocusEvent*)
{
    // only set the focused frame inactive so that we stop painting the caret
    // and the focus frame. But don't tell the focus controller so that upon
    // focusInEvent() we can re-activate the frame.
    FocusController *focusController = page->focusController();
    // Call setFocused first so that window.onblur doesn't get called twice
    focusController->setFocused(false);
    focusController->setActive(false);
}

void QWebPagePrivate::dragEnterEvent(QGraphicsSceneDragDropEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    DragData dragData(ev->mimeData(), ev->pos().toPoint(),
            QCursor::pos(), dropActionToDragOp(ev->possibleActions()));
    Qt::DropAction action = dragOpToDropAction(page->dragController()->dragEntered(&dragData));
    ev->setDropAction(action);
    if (action != Qt::IgnoreAction)
        ev->accept();
#endif
}

void QWebPagePrivate::dragEnterEvent(QDragEnterEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    DragData dragData(ev->mimeData(), ev->pos(), QCursor::pos(),
                      dropActionToDragOp(ev->possibleActions()));
    Qt::DropAction action = dragOpToDropAction(page->dragController()->dragEntered(&dragData));
    ev->setDropAction(action);
    // We must accept this event in order to receive the drag move events that are sent
    // while the drag and drop action is in progress.
    ev->accept();
#endif
}

void QWebPagePrivate::dragLeaveEvent(QGraphicsSceneDragDropEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    DragData dragData(0, IntPoint(), QCursor::pos(), DragOperationNone);
    page->dragController()->dragExited(&dragData);
    ev->accept();
#endif
}

void QWebPagePrivate::dragLeaveEvent(QDragLeaveEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    DragData dragData(0, IntPoint(), QCursor::pos(), DragOperationNone);
    page->dragController()->dragExited(&dragData);
    ev->accept();
#endif
}

void QWebPagePrivate::dragMoveEvent(QGraphicsSceneDragDropEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    DragData dragData(ev->mimeData(), ev->pos().toPoint(),
            QCursor::pos(), dropActionToDragOp(ev->possibleActions()));
    Qt::DropAction action = dragOpToDropAction(page->dragController()->dragUpdated(&dragData));
    ev->setDropAction(action);
    if (action != Qt::IgnoreAction)
        ev->accept();
#endif
}

void QWebPagePrivate::dragMoveEvent(QDragMoveEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    DragData dragData(ev->mimeData(), ev->pos(), QCursor::pos(),
                      dropActionToDragOp(ev->possibleActions()));
    Qt::DropAction action = dragOpToDropAction(page->dragController()->dragUpdated(&dragData));
    m_lastDropAction = action;
    ev->setDropAction(action);
    // We must accept this event in order to receive the drag move events that are sent
    // while the drag and drop action is in progress.
    ev->accept();
#endif
}

void QWebPagePrivate::dropEvent(QGraphicsSceneDragDropEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    DragData dragData(ev->mimeData(), ev->pos().toPoint(),
            QCursor::pos(), dropActionToDragOp(ev->possibleActions()));
    if (page->dragController()->performDrag(&dragData))
        ev->accept();
#endif
}

void QWebPagePrivate::dropEvent(QDropEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    // Overwrite the defaults set by QDragManager::defaultAction()
    ev->setDropAction(m_lastDropAction);
    DragData dragData(ev->mimeData(), ev->pos(), QCursor::pos(),
                      dropActionToDragOp(Qt::DropAction(ev->dropAction())));
    if (page->dragController()->performDrag(&dragData))
        ev->accept();
#endif
}

void QWebPagePrivate::leaveEvent(QEvent*)
{
    // Fake a mouse move event just outside of the widget, since all
    // the interesting mouse-out behavior like invalidating scrollbars
    // is handled by the WebKit event handler's mouseMoved function.
    QMouseEvent fakeEvent(QEvent::MouseMove, QCursor::pos(), Qt::NoButton, Qt::NoButton, Qt::NoModifier);
    mouseMoveEvent(&fakeEvent);
}

void QWebPage::setPalette(const QPalette &pal)
{
    d->palette = pal;
    if (!d->mainFrame || !d->mainFrame->d->frame->view())
        return;

    QBrush brush = pal.brush(QPalette::Base);
    QColor backgroundColor = brush.style() == Qt::SolidPattern ? brush.color() : QColor();
    QWebFramePrivate::core(d->mainFrame)->view()->updateBackgroundRecursively(backgroundColor, !backgroundColor.alpha());
}

QPalette QWebPage::palette() const
{
    return d->palette;
}

void QWebPagePrivate::inputMethodEvent(QInputMethodEvent *ev)
{
    WebCore::Frame *frame = page->focusController()->focusedOrMainFrame();
    WebCore::Editor *editor = frame->editor();

    if (!editor->canEdit()) {
        ev->ignore();
        return;
    }

    RenderObject* renderer = 0;
    RenderTextControl* renderTextControl = 0;

    if (frame->selection()->rootEditableElement())
        renderer = frame->selection()->rootEditableElement()->shadowAncestorNode()->renderer();

    if (renderer && renderer->isTextControl())
        renderTextControl = toRenderTextControl(renderer);

    Vector<CompositionUnderline> underlines;

    for (int i = 0; i < ev->attributes().size(); ++i) {
        const QInputMethodEvent::Attribute& a = ev->attributes().at(i);
        switch (a.type) {
        case QInputMethodEvent::TextFormat: {
            QTextCharFormat textCharFormat = a.value.value<QTextFormat>().toCharFormat();
            QColor qcolor = textCharFormat.underlineColor();
            underlines.append(CompositionUnderline(a.start, a.length, Color(makeRGBA(qcolor.red(), qcolor.green(), qcolor.blue(), qcolor.alpha())), false));
            break;
        }
        case QInputMethodEvent::Cursor: {
            frame->selection()->setCaretVisible(a.length); //if length is 0 cursor is invisible
            if (a.length > 0) {
                RenderObject* caretRenderer = frame->selection()->caretRenderer();
                if (caretRenderer) {
                    QColor qcolor = a.value.value<QColor>();
                    caretRenderer->style()->setColor(Color(makeRGBA(qcolor.red(), qcolor.green(), qcolor.blue(), qcolor.alpha())));
                }
            }
            break;
        }
#if QT_VERSION >= 0x040600
        case QInputMethodEvent::Selection: {
            if (renderTextControl) {
                renderTextControl->setSelectionStart(a.start);
                renderTextControl->setSelectionEnd(a.start + a.length);
            }
            break;
        }
#endif
        }
    }

    if (!ev->commitString().isEmpty())
        editor->confirmComposition(ev->commitString());
    else if (!ev->preeditString().isEmpty()) {
        QString preedit = ev->preeditString();
        editor->setComposition(preedit, underlines, preedit.length(), 0);
    }
    ev->accept();
}

void QWebPagePrivate::shortcutOverrideEvent(QKeyEvent* event)
{
    WebCore::Frame* frame = page->focusController()->focusedOrMainFrame();
    WebCore::Editor* editor = frame->editor();
    if (editor->canEdit()) {
        if (event->modifiers() == Qt::NoModifier
            || event->modifiers() == Qt::ShiftModifier
            || event->modifiers() == Qt::KeypadModifier) {
                if (event->key() < Qt::Key_Escape) {
                    event->accept();
                } else {
                    switch (event->key()) {
                    case Qt::Key_Return:
                    case Qt::Key_Enter:
                    case Qt::Key_Delete:
                    case Qt::Key_Home:
                    case Qt::Key_End:
                    case Qt::Key_Backspace:
                    case Qt::Key_Left:
                    case Qt::Key_Right:
                    case Qt::Key_Up:
                    case Qt::Key_Down:
                    case Qt::Key_Tab:
                        event->accept();
                    default:
                        break;
                    }
                }
        }
#ifndef QT_NO_SHORTCUT
        else if (editorActionForKeyEvent(event) != QWebPage::NoWebAction)
            event->accept();
#endif
    }
}

bool QWebPagePrivate::handleScrolling(QKeyEvent *ev, Frame *frame)
{
    ScrollDirection direction;
    ScrollGranularity granularity;

#ifndef QT_NO_SHORTCUT
    if (ev == QKeySequence::MoveToNextPage
        || (ev->key() == Qt::Key_Space && !(ev->modifiers() & Qt::ShiftModifier))) {
        granularity = ScrollByPage;
        direction = ScrollDown;
    } else if (ev == QKeySequence::MoveToPreviousPage
               || ((ev->key() == Qt::Key_Space) && (ev->modifiers() & Qt::ShiftModifier))) {
        granularity = ScrollByPage;
        direction = ScrollUp;
    } else
#endif // QT_NO_SHORTCUT
    if ((ev->key() == Qt::Key_Up && ev->modifiers() & Qt::ControlModifier)
               || ev->key() == Qt::Key_Home) {
        granularity = ScrollByDocument;
        direction = ScrollUp;
    } else if ((ev->key() == Qt::Key_Down && ev->modifiers() & Qt::ControlModifier)
               || ev->key() == Qt::Key_End) {
        granularity = ScrollByDocument;
        direction = ScrollDown;
    } else {
        switch (ev->key()) {
            case Qt::Key_Up:
                granularity = ScrollByLine;
                direction = ScrollUp;
                break;
            case Qt::Key_Down:
                granularity = ScrollByLine;
                direction = ScrollDown;
                break;
            case Qt::Key_Left:
                granularity = ScrollByLine;
                direction = ScrollLeft;
                break;
            case Qt::Key_Right:
                granularity = ScrollByLine;
                direction = ScrollRight;
                break;
            default:
                return false;
        }
    }

    return frame->eventHandler()->scrollRecursively(direction, granularity);
}

#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
void QWebPagePrivate::touchEvent(QTouchEvent* event)
{
    WebCore::Frame* frame = QWebFramePrivate::core(mainFrame);
    if (!frame->view())
        return;

    bool accepted = frame->eventHandler()->handleTouchEvent(PlatformTouchEvent(event));
    event->setAccepted(accepted);
}
#endif

QVariant QWebPage::inputMethodQuery(Qt::InputMethodQuery property) const
{
    Frame* frame = d->page->focusController()->focusedFrame();
    if (!frame)
        return QVariant();

    WebCore::Editor* editor = frame->editor();

    RenderObject* renderer = 0;
    RenderTextControl* renderTextControl = 0;

    if (frame->selection()->rootEditableElement())
        renderer = frame->selection()->rootEditableElement()->shadowAncestorNode()->renderer();

    if (renderer && renderer->isTextControl())
        renderTextControl = toRenderTextControl(renderer);

    switch (property) {
        case Qt::ImMicroFocus: {
            WebCore::FrameView* view = frame->view();
            if (view && view->needsLayout()) {
                // We can't access absoluteCaretBounds() while the view needs to layout.
                return QVariant();
            }
            return QVariant(frame->selection()->absoluteCaretBounds());
        }
        case Qt::ImFont: {
            if (renderTextControl) {
                RenderStyle* renderStyle = renderTextControl->style();
                return QVariant(QFont(renderStyle->font().font()));
            }
            return QVariant(QFont());
        }
        case Qt::ImCursorPosition: {
            if (renderTextControl) {
                if (editor->hasComposition()) {
                    RefPtr<Range> range = editor->compositionRange();
                    return QVariant(renderTextControl->selectionEnd() - TextIterator::rangeLength(range.get()));
                }
                return QVariant(renderTextControl->selectionEnd());
            }
            return QVariant();
        }
        case Qt::ImSurroundingText: {
            if (renderTextControl) {
                QString text = renderTextControl->text();
                RefPtr<Range> range = editor->compositionRange();
                if (range) {
                    text.remove(range->startPosition().offsetInContainerNode(), TextIterator::rangeLength(range.get()));
                }
                return QVariant(text);
            }
            return QVariant();
        }
        case Qt::ImCurrentSelection: {
            if (renderTextControl) {
                int start = renderTextControl->selectionStart();
                int end = renderTextControl->selectionEnd();
                if (end > start)
                    return QVariant(QString(renderTextControl->text()).mid(start,end-start));
            }
            return QVariant();

        }
#if QT_VERSION >= 0x040600
        case Qt::ImAnchorPosition: {
            if (renderTextControl) {
                if (editor->hasComposition()) {
                    RefPtr<Range> range = editor->compositionRange();
                    return QVariant(renderTextControl->selectionStart() - TextIterator::rangeLength(range.get()));
                }
                return QVariant(renderTextControl->selectionStart());
            }
            return QVariant();
        }
        case Qt::ImMaximumTextLength: {
            if (frame->selection()->isContentEditable()) {
                if (frame->document() && frame->document()->focusedNode()) {
                    if (frame->document()->focusedNode()->hasTagName(HTMLNames::inputTag)) {
                        HTMLInputElement* inputElement = static_cast<HTMLInputElement*>(frame->document()->focusedNode());
                        return QVariant(inputElement->maxLength());
                    }
                }
                return QVariant(InputElement::s_maximumLength);
            }
            return QVariant(0);
        }
#endif
        default:
            return QVariant();
    }
}

void QWebPagePrivate::setInspector(QWebInspector* insp)
{
    if (inspector)
        inspector->d->setFrontend(0);

    if (inspectorIsInternalOnly) {
        QWebInspector* inspToDelete = inspector;
        inspector = 0;
        inspectorIsInternalOnly = false;
        delete inspToDelete;    // Delete after to prevent infinite recursion
    }

    inspector = insp;

    // Give inspector frontend web view if previously created
    if (inspector && inspectorFrontend)
        inspector->d->setFrontend(inspectorFrontend);
}

QWebInspector* QWebPagePrivate::getOrCreateInspector()
{
#if ENABLE(INSPECTOR)
    if (!inspector) {
        QWebInspector* insp = new QWebInspector;
        insp->setPage(q);
        inspectorIsInternalOnly = true;

        Q_ASSERT(inspector); // Associated through QWebInspector::setPage(q)
    }
#endif
    return inspector;
}

/*! \internal */
InspectorController* QWebPagePrivate::inspectorController()
{
#if ENABLE(INSPECTOR)
    return page->inspectorController();
#else
    return 0;
#endif
}








QWebPage::QWebPage(QObject *parent)
    : QObject(parent)
    , d(new QWebPagePrivate(this))
{
    setView(qobject_cast<QWidget *>(parent));

    connect(this, SIGNAL(loadProgress(int)), this, SLOT(_q_onLoadProgressChanged(int)));
#ifndef NDEBUG
    connect(QCoreApplication::instance(), SIGNAL(aboutToQuit()), this, SLOT(_q_cleanupLeakMessages()));
#endif
}

QWebPage::~QWebPage()
{
    d->createMainFrame();
    FrameLoader *loader = d->mainFrame->d->frame->loader();
    if (loader)
        loader->detachFromParent();
    if (d->inspector) {
        // Since we have to delete an internal inspector,
        // call setInspector(0) directly to prevent potential crashes
        if (d->inspectorIsInternalOnly)
            d->setInspector(0);
        else
            d->inspector->setPage(0);
    }
    delete d;
}

QWebFrame *QWebPage::mainFrame() const
{
    d->createMainFrame();
    return d->mainFrame;
}

QWebFrame *QWebPage::currentFrame() const
{
    d->createMainFrame();
    return static_cast<WebCore::FrameLoaderClientQt *>(d->page->focusController()->focusedOrMainFrame()->loader()->client())->webFrame();
}


QWebFrame* QWebPage::frameAt(const QPoint& pos) const
{
    QWebFrame* webFrame = mainFrame();
    if (!webFrame->geometry().contains(pos))
        return 0;
    QWebHitTestResult hitTestResult = webFrame->hitTestContent(pos);
    return hitTestResult.frame();
}

QWebHistory *QWebPage::history() const
{
    d->createMainFrame();
    return &d->history;
}

void QWebPage::setView(QWidget *view)
{
    if (this->view() != view) {
        d->view = view;
        if (!view) {
            delete d->client;
            d->client = 0;
        } else {
            if (!d->client) 
                d->client = new QWebPageWidgetClient(view);
            else
                static_cast<QWebPageWidgetClient*>(d->client)->view = view;
        }
        setViewportSize(view ? view->size() : QSize(0, 0));
    }
}

QWidget *QWebPage::view() const
{
#if QT_VERSION < 0x040600
    return d->view;
#else
    return d->view.data();
#endif
}

void QWebPage::javaScriptConsoleMessage(const QString& message, int lineNumber, const QString& sourceID)
{
    Q_UNUSED(message)
    Q_UNUSED(lineNumber)
    Q_UNUSED(sourceID)
}

void QWebPage::javaScriptAlert(QWebFrame *frame, const QString& msg)
{
    Q_UNUSED(frame)
#ifndef QT_NO_MESSAGEBOX
    QMessageBox::information(view(), tr("JavaScript Alert - %1").arg(mainFrame()->url().host()), Qt::escape(msg), QMessageBox::Ok);
#endif
}

bool QWebPage::javaScriptConfirm(QWebFrame *frame, const QString& msg)
{
    Q_UNUSED(frame)
#ifdef QT_NO_MESSAGEBOX
    return true;
#else
    return QMessageBox::Yes == QMessageBox::information(view(), tr("JavaScript Confirm - %1").arg(mainFrame()->url().host()), Qt::escape(msg), QMessageBox::Yes, QMessageBox::No);
#endif
}

bool QWebPage::javaScriptPrompt(QWebFrame *frame, const QString& msg, const QString& defaultValue, QString* result)
{
    Q_UNUSED(frame)
    bool ok = false;
#ifndef QT_NO_INPUTDIALOG
    QString x = QInputDialog::getText(view(), tr("JavaScript Prompt - %1").arg(mainFrame()->url().host()), Qt::escape(msg), QLineEdit::Normal, defaultValue, &ok);
    if (ok && result)
        *result = x;
#endif
    return ok;
}

bool QWebPage::shouldInterruptJavaScript()
{
#ifdef QT_NO_MESSAGEBOX
    return false;
#else
    return QMessageBox::Yes == QMessageBox::information(view(), tr("JavaScript Problem - %1").arg(mainFrame()->url().host()), tr("The script on this page appears to have a problem. Do you want to stop the script?"), QMessageBox::Yes, QMessageBox::No);
#endif
}

QWebPage *QWebPage::createWindow(WebWindowType type)
{
    QWebView *webView = qobject_cast<QWebView *>(view());
    if (webView) {
        QWebView *newView = webView->createWindow(type);
        if (newView)
            return newView->page();
    }
    return 0;
}

QObject *QWebPage::createPlugin(const QString &classid, const QUrl &url, const QStringList &paramNames, const QStringList &paramValues)
{
    Q_UNUSED(classid)
    Q_UNUSED(url)
    Q_UNUSED(paramNames)
    Q_UNUSED(paramValues)
    return 0;
}

static WebCore::FrameLoadRequest frameLoadRequest(const QUrl &url, WebCore::Frame *frame)
{
    WebCore::ResourceRequest rr(url, frame->loader()->outgoingReferrer());
    return WebCore::FrameLoadRequest(rr);
}

static void openNewWindow(const QUrl& url, WebCore::Frame* frame)
{
    if (Page* oldPage = frame->page()) {
        WindowFeatures features;
        if (Page* newPage = oldPage->chrome()->createWindow(frame,
                frameLoadRequest(url, frame), features))
            newPage->chrome()->show();
    }
}

void QWebPage::triggerAction(WebAction action, bool)
{
    WebCore::Frame *frame = d->page->focusController()->focusedOrMainFrame();
    if (!frame)
        return;
    WebCore::Editor *editor = frame->editor();
    const char *command = 0;

    switch (action) {
        case OpenLink:
            if (QWebFrame *targetFrame = d->hitTestResult.linkTargetFrame()) {
                WTF::RefPtr<WebCore::Frame> wcFrame = targetFrame->d->frame;
                targetFrame->d->frame->loader()->loadFrameRequest(frameLoadRequest(d->hitTestResult.linkUrl(), wcFrame.get()),
                                                                  /*lockHistory*/ false, /*lockBackForwardList*/ false, /*event*/ 0,
                                                                  /*FormState*/ 0, SendReferrer);
                break;
            }
            // fall through
        case OpenLinkInNewWindow:
            openNewWindow(d->hitTestResult.linkUrl(), frame);
            break;
        case OpenFrameInNewWindow: {
            KURL url = frame->loader()->documentLoader()->unreachableURL();
            if (url.isEmpty())
                url = frame->loader()->documentLoader()->url();
            openNewWindow(url, frame);
            break;
        }
        case CopyLinkToClipboard: {
#if defined(Q_WS_X11)
            bool oldSelectionMode = Pasteboard::generalPasteboard()->isSelectionMode();
            Pasteboard::generalPasteboard()->setSelectionMode(true);
            editor->copyURL(d->hitTestResult.linkUrl(), d->hitTestResult.linkText());
            Pasteboard::generalPasteboard()->setSelectionMode(oldSelectionMode);
#endif
            editor->copyURL(d->hitTestResult.linkUrl(), d->hitTestResult.linkText());
            break;
        }
        case OpenImageInNewWindow:
            openNewWindow(d->hitTestResult.imageUrl(), frame);
            break;
        case DownloadImageToDisk:
            frame->loader()->client()->startDownload(WebCore::ResourceRequest(d->hitTestResult.imageUrl(), frame->loader()->outgoingReferrer()));
            break;
        case DownloadLinkToDisk:
            frame->loader()->client()->startDownload(WebCore::ResourceRequest(d->hitTestResult.linkUrl(), frame->loader()->outgoingReferrer()));
            break;
#ifndef QT_NO_CLIPBOARD
        case CopyImageToClipboard:
            QApplication::clipboard()->setPixmap(d->hitTestResult.pixmap());
            break;
#endif
        case Back:
            d->page->goBack();
            break;
        case Forward:
            d->page->goForward();
            break;
        case Stop:
            mainFrame()->d->frame->loader()->stopForUserCancel();
            break;
        case Reload:
            mainFrame()->d->frame->loader()->reload(/*endtoendreload*/false);
            break;
        case ReloadAndBypassCache:
            mainFrame()->d->frame->loader()->reload(/*endtoendreload*/true);
            break;
        case SetTextDirectionDefault:
            editor->setBaseWritingDirection(NaturalWritingDirection);
            break;
        case SetTextDirectionLeftToRight:
            editor->setBaseWritingDirection(LeftToRightWritingDirection);
            break;
        case SetTextDirectionRightToLeft:
            editor->setBaseWritingDirection(RightToLeftWritingDirection);
            break;
        case InspectElement: {
#if ENABLE(INSPECTOR)
            if (!d->hitTestResult.isNull()) {
                d->getOrCreateInspector(); // Make sure the inspector is created
                d->inspector->show(); // The inspector is expected to be shown on inspection
                d->page->inspectorController()->inspect(d->hitTestResult.d->innerNonSharedNode.get());
            }
#endif
            break;
        }
        default:
            command = QWebPagePrivate::editorCommandForWebActions(action);
            break;
    }

    if (command)
        editor->command(command).execute();
}

QSize QWebPage::viewportSize() const
{
    if (d->mainFrame && d->mainFrame->d->frame->view())
        return d->mainFrame->d->frame->view()->frameRect().size();

    return d->viewportSize;
}

void QWebPage::setViewportSize(const QSize &size) const
{
    d->viewportSize = size;

    QWebFrame *frame = mainFrame();
    if (frame->d->frame && frame->d->frame->view()) {
        WebCore::FrameView* view = frame->d->frame->view();
        view->setFrameRect(QRect(QPoint(0, 0), size));
        view->forceLayout();
        view->adjustViewSize();
    }
}

QSize QWebPage::preferredContentsSize() const
{
    QWebFrame* frame = d->mainFrame;
    if (frame) {
        WebCore::FrameView* view = frame->d->frame->view();
        if (view && view->useFixedLayout())
            return d->mainFrame->d->frame->view()->fixedLayoutSize();
    }

    return d->fixedLayoutSize;
}

void QWebPage::setPreferredContentsSize(const QSize &size) const
{
    d->fixedLayoutSize = size;

    QWebFrame *frame = mainFrame();
    if (frame->d->frame && frame->d->frame->view()) {
        WebCore::FrameView* view = frame->d->frame->view();

        if (size.isValid()) {
            view->setUseFixedLayout(true);
            view->setFixedLayoutSize(size);
            view->forceLayout();
        } else if (view->useFixedLayout()) {
            view->setUseFixedLayout(false);
            view->forceLayout();
        }
    }
}

bool QWebPage::acceptNavigationRequest(QWebFrame *frame, const QNetworkRequest &request, QWebPage::NavigationType type)
{
    Q_UNUSED(frame)
    if (type == NavigationTypeLinkClicked) {
        switch (d->linkPolicy) {
            case DontDelegateLinks:
                return true;

            case DelegateExternalLinks:
                if (WebCore::SecurityOrigin::shouldTreatURLSchemeAsLocal(request.url().scheme()))
                    return true;
                emit linkClicked(request.url());
                return false;

            case DelegateAllLinks:
                emit linkClicked(request.url());
                return false;
        }
    }
    return true;
}

QString QWebPage::selectedText() const
{
    d->createMainFrame();
    return d->page->focusController()->focusedOrMainFrame()->selectedText();
}

QAction *QWebPage::action(WebAction action) const
{
    if (action == QWebPage::NoWebAction) return 0;
    if (d->actions[action])
        return d->actions[action];

    QString text;
    QIcon icon;
    QStyle *style = view() ? view()->style() : qApp->style();
    bool checkable = false;

    switch (action) {
        case OpenLink:
            text = contextMenuItemTagOpenLink();
            break;
        case OpenLinkInNewWindow:
            text = contextMenuItemTagOpenLinkInNewWindow();
            break;
        case OpenFrameInNewWindow:
            text = contextMenuItemTagOpenFrameInNewWindow();
            break;

        case DownloadLinkToDisk:
            text = contextMenuItemTagDownloadLinkToDisk();
            break;
        case CopyLinkToClipboard:
            text = contextMenuItemTagCopyLinkToClipboard();
            break;

        case OpenImageInNewWindow:
            text = contextMenuItemTagOpenImageInNewWindow();
            break;
        case DownloadImageToDisk:
            text = contextMenuItemTagDownloadImageToDisk();
            break;
        case CopyImageToClipboard:
            text = contextMenuItemTagCopyImageToClipboard();
            break;

        case Back:
            text = contextMenuItemTagGoBack();
            icon = style->standardIcon(QStyle::SP_ArrowBack);
            break;
        case Forward:
            text = contextMenuItemTagGoForward();
            icon = style->standardIcon(QStyle::SP_ArrowForward);
            break;
        case Stop:
            text = contextMenuItemTagStop();
            icon = style->standardIcon(QStyle::SP_BrowserStop);
            break;
        case Reload:
            text = contextMenuItemTagReload();
            icon = style->standardIcon(QStyle::SP_BrowserReload);
            break;

        case Cut:
            text = contextMenuItemTagCut();
            break;
        case Copy:
            text = contextMenuItemTagCopy();
            break;
        case Paste:
            text = contextMenuItemTagPaste();
            break;
#ifndef QT_NO_UNDOSTACK
        case Undo: {
            QAction *a = undoStack()->createUndoAction(d->q);
            d->actions[action] = a;
            return a;
        }
        case Redo: {
            QAction *a = undoStack()->createRedoAction(d->q);
            d->actions[action] = a;
            return a;
        }
#endif // QT_NO_UNDOSTACK
        case MoveToNextChar:
            text = tr("Move the cursor to the next character");
            break;
        case MoveToPreviousChar:
            text = tr("Move the cursor to the previous character");
            break;
        case MoveToNextWord:
            text = tr("Move the cursor to the next word");
            break;
        case MoveToPreviousWord:
            text = tr("Move the cursor to the previous word");
            break;
        case MoveToNextLine:
            text = tr("Move the cursor to the next line");
            break;
        case MoveToPreviousLine:
            text = tr("Move the cursor to the previous line");
            break;
        case MoveToStartOfLine:
            text = tr("Move the cursor to the start of the line");
            break;
        case MoveToEndOfLine:
            text = tr("Move the cursor to the end of the line");
            break;
        case MoveToStartOfBlock:
            text = tr("Move the cursor to the start of the block");
            break;
        case MoveToEndOfBlock:
            text = tr("Move the cursor to the end of the block");
            break;
        case MoveToStartOfDocument:
            text = tr("Move the cursor to the start of the document");
            break;
        case MoveToEndOfDocument:
            text = tr("Move the cursor to the end of the document");
            break;
        case SelectAll:
            text = tr("Select all");
            break;
        case SelectNextChar:
            text = tr("Select to the next character");
            break;
        case SelectPreviousChar:
            text = tr("Select to the previous character");
            break;
        case SelectNextWord:
            text = tr("Select to the next word");
            break;
        case SelectPreviousWord:
            text = tr("Select to the previous word");
            break;
        case SelectNextLine:
            text = tr("Select to the next line");
            break;
        case SelectPreviousLine:
            text = tr("Select to the previous line");
            break;
        case SelectStartOfLine:
            text = tr("Select to the start of the line");
            break;
        case SelectEndOfLine:
            text = tr("Select to the end of the line");
            break;
        case SelectStartOfBlock:
            text = tr("Select to the start of the block");
            break;
        case SelectEndOfBlock:
            text = tr("Select to the end of the block");
            break;
        case SelectStartOfDocument:
            text = tr("Select to the start of the document");
            break;
        case SelectEndOfDocument:
            text = tr("Select to the end of the document");
            break;
        case DeleteStartOfWord:
            text = tr("Delete to the start of the word");
            break;
        case DeleteEndOfWord:
            text = tr("Delete to the end of the word");
            break;

        case SetTextDirectionDefault:
            text = contextMenuItemTagDefaultDirection();
            break;
        case SetTextDirectionLeftToRight:
            text = contextMenuItemTagLeftToRight();
            checkable = true;
            break;
        case SetTextDirectionRightToLeft:
            text = contextMenuItemTagRightToLeft();
            checkable = true;
            break;

        case ToggleBold:
            text = contextMenuItemTagBold();
            checkable = true;
            break;
        case ToggleItalic:
            text = contextMenuItemTagItalic();
            checkable = true;
            break;
        case ToggleUnderline:
            text = contextMenuItemTagUnderline();
            checkable = true;
            break;

        case InspectElement:
            text = contextMenuItemTagInspectElement();
            break;

        case InsertParagraphSeparator:
            text = tr("Insert a new paragraph");
            break;
        case InsertLineSeparator:
            text = tr("Insert a new line");
            break;

        case PasteAndMatchStyle:
            text = tr("Paste and Match Style");
            break;
        case RemoveFormat:
            text = tr("Remove formatting");
            break;

        case ToggleStrikethrough:
            text = tr("Strikethrough");
            checkable = true;
            break;
        case ToggleSubscript:
            text = tr("Subscript");
            checkable = true;
            break;
        case ToggleSuperscript:
            text = tr("Superscript");
            checkable = true;
            break;
        case InsertUnorderedList:
            text = tr("Insert Bulleted List");
            checkable = true;
            break;
        case InsertOrderedList:
            text = tr("Insert Numbered List");
            checkable = true;
            break;
        case Indent:
            text = tr("Indent");
            break;
        case Outdent:
            text = tr("Outdent");
            break;
        case AlignCenter:
            text = tr("Center");
            break;
        case AlignJustified:
            text = tr("Justify");
            break;
        case AlignLeft:
            text = tr("Align Left");
            break;
        case AlignRight:
            text = tr("Align Right");
            break;

        case NoWebAction:
            return 0;
    }

    if (text.isEmpty())
        return 0;

    QAction *a = new QAction(d->q);
    a->setText(text);
    a->setData(action);
    a->setCheckable(checkable);
    a->setIcon(icon);

    connect(a, SIGNAL(triggered(bool)),
            this, SLOT(_q_webActionTriggered(bool)));

    d->actions[action] = a;
    d->updateAction(action);
    return a;
}

bool QWebPage::isModified() const
{
#ifdef QT_NO_UNDOSTACK
    return false;
#else
    if (!d->undoStack)
        return false;
    return d->undoStack->canUndo();
#endif // QT_NO_UNDOSTACK
}

#ifndef QT_NO_UNDOSTACK
QUndoStack *QWebPage::undoStack() const
{
    if (!d->undoStack)
        d->undoStack = new QUndoStack(const_cast<QWebPage *>(this));

    return d->undoStack;
}
#endif // QT_NO_UNDOSTACK

bool QWebPage::event(QEvent *ev)
{
    switch (ev->type()) {
    case QEvent::Timer:
        d->timerEvent(static_cast<QTimerEvent*>(ev));
        break;
    case QEvent::MouseMove:
        d->mouseMoveEvent(static_cast<QMouseEvent*>(ev));
        break;
    case QEvent::GraphicsSceneMouseMove:
        d->mouseMoveEvent(static_cast<QGraphicsSceneMouseEvent*>(ev));
        break;
    case QEvent::MouseButtonPress:
        d->mousePressEvent(static_cast<QMouseEvent*>(ev));
        break;
    case QEvent::GraphicsSceneMousePress:
        d->mousePressEvent(static_cast<QGraphicsSceneMouseEvent*>(ev));
        break;
    case QEvent::MouseButtonDblClick:
        d->mouseDoubleClickEvent(static_cast<QMouseEvent*>(ev));
        break;
    case QEvent::GraphicsSceneMouseDoubleClick:
        d->mouseDoubleClickEvent(static_cast<QGraphicsSceneMouseEvent*>(ev));
        break;
    case QEvent::MouseButtonRelease:
        d->mouseReleaseEvent(static_cast<QMouseEvent*>(ev));
        break;
    case QEvent::GraphicsSceneMouseRelease:
        d->mouseReleaseEvent(static_cast<QGraphicsSceneMouseEvent*>(ev));
        break;
#ifndef QT_NO_CONTEXTMENU
    case QEvent::ContextMenu:
        d->contextMenuEvent(static_cast<QContextMenuEvent*>(ev)->globalPos());
        break;
    case QEvent::GraphicsSceneContextMenu:
        d->contextMenuEvent(static_cast<QGraphicsSceneContextMenuEvent*>(ev)->screenPos());
        break;
#endif
#ifndef QT_NO_WHEELEVENT
    case QEvent::Wheel:
        d->wheelEvent(static_cast<QWheelEvent*>(ev));
        break;
    case QEvent::GraphicsSceneWheel:
        d->wheelEvent(static_cast<QGraphicsSceneWheelEvent*>(ev));
        break;
#endif
    case QEvent::KeyPress:
        d->keyPressEvent(static_cast<QKeyEvent*>(ev));
        break;
    case QEvent::KeyRelease:
        d->keyReleaseEvent(static_cast<QKeyEvent*>(ev));
        break;
    case QEvent::FocusIn:
        d->focusInEvent(static_cast<QFocusEvent*>(ev));
        break;
    case QEvent::FocusOut:
        d->focusOutEvent(static_cast<QFocusEvent*>(ev));
        break;
#ifndef QT_NO_DRAGANDDROP
    case QEvent::DragEnter:
        d->dragEnterEvent(static_cast<QDragEnterEvent*>(ev));
        break;
    case QEvent::GraphicsSceneDragEnter:
        d->dragEnterEvent(static_cast<QGraphicsSceneDragDropEvent*>(ev));
        break;
    case QEvent::DragLeave:
        d->dragLeaveEvent(static_cast<QDragLeaveEvent*>(ev));
        break;
    case QEvent::GraphicsSceneDragLeave:
        d->dragLeaveEvent(static_cast<QGraphicsSceneDragDropEvent*>(ev));
        break;
    case QEvent::DragMove:
        d->dragMoveEvent(static_cast<QDragMoveEvent*>(ev));
        break;
    case QEvent::GraphicsSceneDragMove:
        d->dragMoveEvent(static_cast<QGraphicsSceneDragDropEvent*>(ev));
        break;
    case QEvent::Drop:
        d->dropEvent(static_cast<QDropEvent*>(ev));
        break;
    case QEvent::GraphicsSceneDrop:
        d->dropEvent(static_cast<QGraphicsSceneDragDropEvent*>(ev));
        break;
#endif
    case QEvent::InputMethod:
        d->inputMethodEvent(static_cast<QInputMethodEvent*>(ev));
    case QEvent::ShortcutOverride:
        d->shortcutOverrideEvent(static_cast<QKeyEvent*>(ev));
        break;
    case QEvent::Leave:
        d->leaveEvent(ev);
        break;
#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
    case QEvent::TouchEnd:
        d->touchEvent(static_cast<QTouchEvent*>(ev));
        break;
#endif
    default:
        return QObject::event(ev);
    }

    return true;
}

bool QWebPage::focusNextPrevChild(bool next)
{
    QKeyEvent ev(QEvent::KeyPress, Qt::Key_Tab, Qt::KeyboardModifiers(next ? Qt::NoModifier : Qt::ShiftModifier));
    d->keyPressEvent(&ev);
    bool hasFocusedNode = false;
    Frame *frame = d->page->focusController()->focusedFrame();
    if (frame) {
        Document *document = frame->document();
        hasFocusedNode = document && document->focusedNode();
    }
    //qDebug() << "focusNextPrevChild(" << next << ") =" << ev.isAccepted() << "focusedNode?" << hasFocusedNode;
    return hasFocusedNode;
}

void QWebPage::setContentEditable(bool editable)
{
    if (d->editable != editable) {
        d->editable = editable;
        d->page->setTabKeyCyclesThroughElements(!editable);
        if (d->mainFrame) {
            WebCore::Frame* frame = d->mainFrame->d->frame;
            if (editable) {
                frame->applyEditingStyleToBodyElement();
                // FIXME: mac port calls this if there is no selectedDOMRange
                //frame->setSelectionFromNone();
            } else
                frame->removeEditingStyleFromBodyElement();
        }

        d->updateEditorActions();
    }
}

bool QWebPage::isContentEditable() const
{
    return d->editable;
}


void QWebPage::setForwardUnsupportedContent(bool forward)
{
    d->forwardUnsupportedContent = forward;
}

bool QWebPage::forwardUnsupportedContent() const
{
    return d->forwardUnsupportedContent;
}


void QWebPage::setLinkDelegationPolicy(LinkDelegationPolicy policy)
{
    d->linkPolicy = policy;
}

QWebPage::LinkDelegationPolicy QWebPage::linkDelegationPolicy() const
{
    return d->linkPolicy;
}

#ifndef QT_NO_CONTEXTMENU
bool QWebPage::swallowContextMenuEvent(QContextMenuEvent *event)
{
    d->page->contextMenuController()->clearContextMenu();

    if (QWebFrame* webFrame = frameAt(event->pos())) {
        Frame* frame = QWebFramePrivate::core(webFrame);
        if (Scrollbar* scrollbar = frame->view()->scrollbarAtPoint(PlatformMouseEvent(event, 1).pos()))
            return scrollbar->contextMenu(PlatformMouseEvent(event, 1));
    }

    WebCore::Frame* focusedFrame = d->page->focusController()->focusedOrMainFrame();
    focusedFrame->eventHandler()->sendContextMenuEvent(PlatformMouseEvent(event, 1));
    ContextMenu *menu = d->page->contextMenuController()->contextMenu();
    // If the website defines its own handler then sendContextMenuEvent takes care of
    // calling/showing it and the context menu pointer will be zero. This is the case
    // on maps.google.com for example.

    return !menu;
}
#endif // QT_NO_CONTEXTMENU

void QWebPage::updatePositionDependentActions(const QPoint &pos)
{
    // First we disable all actions, but keep track of which ones were originally enabled.
    QBitArray originallyEnabledWebActions(QWebPage::WebActionCount);
    for (int i = ContextMenuItemTagNoAction; i < ContextMenuItemBaseApplicationTag; ++i) {
        QWebPage::WebAction action = webActionForContextMenuAction(WebCore::ContextMenuAction(i));
        if (QAction *a = this->action(action)) {
            originallyEnabledWebActions.setBit(action, a->isEnabled());
            a->setEnabled(false);
        }
    }

    d->createMainFrame();
    WebCore::Frame* focusedFrame = d->page->focusController()->focusedOrMainFrame();
    HitTestResult result = focusedFrame->eventHandler()->hitTestResultAtPoint(focusedFrame->view()->windowToContents(pos), /*allowShadowContent*/ false);

    if (result.scrollbar())
        d->hitTestResult = QWebHitTestResult();
    else
        d->hitTestResult = QWebHitTestResult(new QWebHitTestResultPrivate(result));
    WebCore::ContextMenu menu(result);
    menu.populate();
    
#if ENABLE(INSPECTOR)
    if (d->page->inspectorController()->enabled())
        menu.addInspectElementItem();
#endif

    QBitArray visitedWebActions(QWebPage::WebActionCount);

#ifndef QT_NO_CONTEXTMENU
    delete d->currentContextMenu;

    // Then we let createContextMenu() enable the actions that are put into the menu
    d->currentContextMenu = d->createContextMenu(&menu, menu.platformDescription(), &visitedWebActions);
#endif // QT_NO_CONTEXTMENU

    // Finally, we restore the original enablement for the actions that were not put into the menu.
    originallyEnabledWebActions &= ~visitedWebActions; // Mask out visited actions (they're part of the menu)
    for (int i = 0; i < QWebPage::WebActionCount; ++i) {
        if (originallyEnabledWebActions.at(i)) {
            if (QAction *a = this->action(QWebPage::WebAction(i)))
                a->setEnabled(true);
        }
    }

    // This whole process ensures that any actions put into to the context menu has the right
    // enablement, while also keeping the correct enablement for actions that were left out of
    // the menu.

}











bool QWebPage::extension(Extension extension, const ExtensionOption *option, ExtensionReturn *output)
{
#ifndef QT_NO_FILEDIALOG
    if (extension == ChooseMultipleFilesExtension) {
        // FIXME: do not ignore suggestedFiles
        QStringList suggestedFiles = static_cast<const ChooseMultipleFilesExtensionOption*>(option)->suggestedFileNames;
        QStringList names = QFileDialog::getOpenFileNames(view(), QString::null);
        static_cast<ChooseMultipleFilesExtensionReturn*>(output)->fileNames = names;
        return true;
    }
#endif

    return false;
}

bool QWebPage::supportsExtension(Extension extension) const
{
#ifndef QT_NO_FILEDIALOG
    return extension == ChooseMultipleFilesExtension;
#else
    Q_UNUSED(extension);
    return false;
#endif
}

bool QWebPage::findText(const QString &subString, FindFlags options)
{
    ::TextCaseSensitivity caseSensitivity = ::TextCaseInsensitive;
    if (options & FindCaseSensitively)
        caseSensitivity = ::TextCaseSensitive;

    if (options & HighlightAllOccurrences) {
        if (subString.isEmpty()) {
            d->page->unmarkAllTextMatches();
            return true;
        } else
            return d->page->markAllMatchesForText(subString, caseSensitivity, true, 0);
    } else {
        ::FindDirection direction = ::FindDirectionForward;
        if (options & FindBackward)
            direction = ::FindDirectionBackward;

        const bool shouldWrap = options & FindWrapsAroundDocument;

        return d->page->findString(subString, caseSensitivity, direction, shouldWrap);
    }
}

QWebSettings *QWebPage::settings() const
{
    return d->settings;
}

QString QWebPage::chooseFile(QWebFrame *parentFrame, const QString& suggestedFile)
{
    Q_UNUSED(parentFrame)
#ifndef QT_NO_FILEDIALOG
    return QFileDialog::getOpenFileName(view(), QString::null, suggestedFile);
#else
    return QString::null;
#endif
}

void QWebPage::setNetworkAccessManager(QNetworkAccessManager *manager)
{
    if (manager == d->networkManager)
        return;
    if (d->networkManager && d->networkManager->parent() == this)
        delete d->networkManager;
    d->networkManager = manager;
}

QNetworkAccessManager *QWebPage::networkAccessManager() const
{
    if (!d->networkManager) {
        QWebPage *that = const_cast<QWebPage *>(this);
        that->d->networkManager = new QNetworkAccessManager(that);
    }
    return d->networkManager;
}

void QWebPage::setPluginFactory(QWebPluginFactory *factory)
{
    d->pluginFactory = factory;
}

QWebPluginFactory *QWebPage::pluginFactory() const
{
    return d->pluginFactory;
}

QString QWebPage::userAgentForUrl(const QUrl& url) const
{
    Q_UNUSED(url)
    QString ua = QLatin1String("Mozilla/5.0 ("

    // Platform
#ifdef Q_WS_MAC
    "Macintosh"
#elif defined Q_WS_QWS
    "QtEmbedded"
#elif defined Q_WS_WIN
    "Windows"
#elif defined Q_WS_X11
    "X11"
#elif defined Q_OS_SYMBIAN
    "SymbianOS"
#else
    "Unknown"
#endif
    // Placeholder for Platform Version
    "%1; "

    // Placeholder for security strength (N or U)
    "%2; "

    // Subplatform"
#ifdef Q_OS_AIX
    "AIX"
#elif defined Q_OS_WIN32
    "%3"
#elif defined Q_OS_DARWIN
#ifdef __i386__ || __x86_64__
    "Intel Mac OS X"
#else
    "PPC Mac OS X"
#endif

#elif defined Q_OS_BSDI
    "BSD"
#elif defined Q_OS_BSD4
    "BSD Four"
#elif defined Q_OS_CYGWIN
    "Cygwin"
#elif defined Q_OS_DGUX
    "DG/UX"
#elif defined Q_OS_DYNIX
    "DYNIX/ptx"
#elif defined Q_OS_FREEBSD
    "FreeBSD"
#elif defined Q_OS_HPUX
    "HP-UX"
#elif defined Q_OS_HURD
    "GNU Hurd"
#elif defined Q_OS_IRIX
    "SGI Irix"
#elif defined Q_OS_LINUX
    "Linux"
#elif defined Q_OS_LYNX
    "LynxOS"
#elif defined Q_OS_NETBSD
    "NetBSD"
#elif defined Q_OS_OS2
    "OS/2"
#elif defined Q_OS_OPENBSD
    "OpenBSD"
#elif defined Q_OS_OS2EMX
    "OS/2"
#elif defined Q_OS_OSF
    "HP Tru64 UNIX"
#elif defined Q_OS_QNX6
    "QNX RTP Six"
#elif defined Q_OS_QNX
    "QNX"
#elif defined Q_OS_RELIANT
    "Reliant UNIX"
#elif defined Q_OS_SCO
    "SCO OpenServer"
#elif defined Q_OS_SOLARIS
    "Sun Solaris"
#elif defined Q_OS_ULTRIX
    "DEC Ultrix"
#elif defined Q_WS_S60
    "Series60"
#elif defined Q_OS_UNIX
    "UNIX BSD/SYSV system"
#elif defined Q_OS_UNIXWARE
    "UnixWare Seven, Open UNIX Eight"
#else
    "Unknown"
#endif
    // Placeholder for SubPlatform Version
    "%4; ");

    // Platform Version
    QString osVer;
#ifdef Q_OS_SYMBIAN
    QSysInfo::SymbianVersion symbianVersion = QSysInfo::symbianVersion();
    switch (symbianVersion) {
    case QSysInfo::SV_9_2:
        osVer = "/9.2";
        break;
    case QSysInfo::SV_9_3:
        osVer = "/9.3";
        break;
    case QSysInfo::SV_9_4:
        osVer = "/9.4";
        break;
    default: 
        osVer = "Unknown";
    }
#endif
    ua = ua.arg(osVer);

    QChar securityStrength(QLatin1Char('N'));
#if !defined(QT_NO_OPENSSL)
    // we could check QSslSocket::supportsSsl() here, but this makes
    // OpenSSL, certificates etc being loaded in all cases were QWebPage
    // is used. This loading is not needed for non-https.
    securityStrength = QLatin1Char('U');
    // this may lead to a false positive: We indicate SSL since it is
    // compiled in even though supportsSsl() might return false
#endif
    ua = ua.arg(securityStrength);

#if defined Q_OS_WIN32
    QString ver;
    switch (QSysInfo::WindowsVersion) {
        case QSysInfo::WV_32s:
            ver = "Windows 3.1";
            break;
        case QSysInfo::WV_95:
            ver = "Windows 95";
            break;
        case QSysInfo::WV_98:
            ver = "Windows 98";
            break;
        case QSysInfo::WV_Me:
            ver = "Windows 98; Win 9x 4.90";
            break;
        case QSysInfo::WV_NT:
            ver = "WinNT4.0";
            break;
        case QSysInfo::WV_2000:
            ver = "Windows NT 5.0";
            break;
        case QSysInfo::WV_XP:
            ver = "Windows NT 5.1";
            break;
        case QSysInfo::WV_2003:
            ver = "Windows NT 5.2";
            break;
        case QSysInfo::WV_VISTA:
            ver = "Windows NT 6.0";
            break;
#if QT_VERSION > 0x040500
        case QSysInfo::WV_WINDOWS7:
            ver = "Windows NT 6.1";
            break;
#endif
        case QSysInfo::WV_CE:
            ver = "Windows CE";
            break;
        case QSysInfo::WV_CENET:
            ver = "Windows CE .NET";
            break;
        case QSysInfo::WV_CE_5:
            ver = "Windows CE 5.x";
            break;
        case QSysInfo::WV_CE_6:
            ver = "Windows CE 6.x";
            break;
    }
    ua = QString(ua).arg(ver);
#endif

    // SubPlatform Version
    QString subPlatformVer;
#ifdef Q_OS_SYMBIAN
    QSysInfo::S60Version s60Version = QSysInfo::s60Version();
    switch (s60Version) {
    case QSysInfo::SV_S60_3_1:
        subPlatformVer = "/3.1";
        break;
    case QSysInfo::SV_S60_3_2:
        subPlatformVer = "/3.2";
        break;
    case QSysInfo::SV_S60_5_0:
        subPlatformVer = "/5.0";
        break;
    default: 
        subPlatformVer = " Unknown";
    }
#endif
    ua = ua.arg(subPlatformVer);

    // Language
    QLocale locale;
    if (view())
        locale = view()->locale();
    QString name = locale.name();
    name[2] = QLatin1Char('-');
    ua.append(name);
    ua.append(QLatin1String(") "));

    // webkit/qt version
    ua.append(QString(QLatin1String("AppleWebKit/%1 (KHTML, like Gecko) "))
                      .arg(QString(qWebKitVersion())));

    // Application name/version
    QString appName = QCoreApplication::applicationName();
    if (!appName.isEmpty()) {
        ua.append(appName);
        QString appVer = QCoreApplication::applicationVersion();
        if (!appVer.isEmpty())
            ua.append(QLatin1Char('/') + appVer);
    } else {
        // Qt version
        ua.append(QLatin1String("Qt/"));
        ua.append(QLatin1String(qVersion()));
    }

    ua.append(QString(QLatin1String(" Safari/%1"))
                      .arg(qWebKitVersion()));

    return ua;
}


void QWebPagePrivate::_q_onLoadProgressChanged(int)
{
    m_totalBytes = page->progress()->totalPageAndResourceBytesToLoad();
    m_bytesReceived = page->progress()->totalBytesReceived();
}


quint64 QWebPage::totalBytes() const
{
    return d->m_totalBytes;
}


quint64 QWebPage::bytesReceived() const
{
    return d->m_bytesReceived;
}

























#include "moc_qwebpage.cpp"
