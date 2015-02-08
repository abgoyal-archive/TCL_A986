

#include "mainwindow.h"

#include "locationedit.h"
#include "utils.h"

MainWindow::MainWindow(const QString& url)
    : m_page(new WebPage(this))
{
    setAttribute(Qt::WA_DeleteOnClose);
#if QT_VERSION >= QT_VERSION_CHECK(4, 5, 0)
    if (qgetenv("QTLAUNCHER_USE_ARGB_VISUALS").toInt() == 1)
        setAttribute(Qt::WA_TranslucentBackground);
#endif

    buildUI();
}

void MainWindow::buildUI()
{
    QToolBar* bar = addToolBar("Navigation");
#if defined(Q_WS_S60)
    bar->setIconSize(QSize(16, 16));
#endif
    bar->addAction(page()->action(QWebPage::Back));
    bar->addAction(page()->action(QWebPage::Forward));
    bar->addAction(page()->action(QWebPage::Reload));
    bar->addAction(page()->action(QWebPage::Stop));

    urlEdit = new LocationEdit(this);
    urlEdit->setSizePolicy(QSizePolicy::Expanding, urlEdit->sizePolicy().verticalPolicy());
    connect(urlEdit, SIGNAL(returnPressed()), SLOT(changeLocation()));
    QCompleter* completer = new QCompleter(this);
    urlEdit->setCompleter(completer);
    completer->setModel(&urlModel);
#if defined(Q_WS_S60)
    addToolBarBreak();
    addToolBar("Location")->addWidget(urlEdit);
#else
    bar->addWidget(urlEdit);
#endif

    connect(page()->mainFrame(), SIGNAL(titleChanged(const QString&)),
            this, SLOT(setWindowTitle(const QString&)));
    connect(page(), SIGNAL(loadProgress(int)), urlEdit, SLOT(setProgress(int)));
    connect(page(), SIGNAL(windowCloseRequested()), this, SLOT(close()));

    // short-cuts
    page()->action(QWebPage::Back)->setShortcut(QKeySequence::Back);
    page()->action(QWebPage::Stop)->setShortcut(Qt::Key_Escape);
    page()->action(QWebPage::Forward)->setShortcut(QKeySequence::Forward);
    page()->action(QWebPage::Reload)->setShortcut(QKeySequence::Refresh);
    page()->action(QWebPage::Undo)->setShortcut(QKeySequence::Undo);
    page()->action(QWebPage::Redo)->setShortcut(QKeySequence::Redo);
    page()->action(QWebPage::Cut)->setShortcut(QKeySequence::Cut);
    page()->action(QWebPage::Copy)->setShortcut(QKeySequence::Copy);
    page()->action(QWebPage::Paste)->setShortcut(QKeySequence::Paste);

    page()->action(QWebPage::ToggleBold)->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_B));
    page()->action(QWebPage::ToggleItalic)->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_I));
    page()->action(QWebPage::ToggleUnderline)->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_U));
}

WebPage* MainWindow::page()
{
    return m_page;
}

void MainWindow::setAddressUrl(const QString& url)
{
    urlEdit->setText(url);
}

void MainWindow::addCompleterEntry(const QUrl& url)
{
    QUrl::FormattingOptions opts;
    opts |= QUrl::RemoveScheme;
    opts |= QUrl::RemoveUserInfo;
    opts |= QUrl::StripTrailingSlash;
    QString s = url.toString(opts);
    s = s.mid(2);
    if (s.isEmpty())
        return;

    if (!urlList.contains(s))
        urlList += s;
    urlModel.setStringList(urlList);
}

void MainWindow::load(const QString& url)
{
    QUrl qurl = urlFromUserInput(url);
    if (qurl.scheme().isEmpty())
        qurl = QUrl("http://" + url + "/");
    load(qurl);
}

void MainWindow::load(const QUrl& url)
{
    if (!url.isValid())
        return;

    setAddressUrl(url.toString());
    page()->mainFrame()->load(url);
}

void MainWindow::changeLocation()
{
    QString string = urlEdit->text();
    load(string);
}

void MainWindow::openFile()
{
    static const QString filter("HTML Files (*.htm *.html);;Text Files (*.txt);;Image Files (*.gif *.jpg *.png);;All Files (*)");

    QFileDialog fileDialog(this, tr("Open"), QString(), filter);
    fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
    fileDialog.setFileMode(QFileDialog::ExistingFile);
    fileDialog.setOptions(QFileDialog::ReadOnly);

    if (fileDialog.exec()) {
        QString selectedFile = fileDialog.selectedFiles()[0];
        if (!selectedFile.isEmpty())
            load(QUrl::fromLocalFile(selectedFile));
    }
}

