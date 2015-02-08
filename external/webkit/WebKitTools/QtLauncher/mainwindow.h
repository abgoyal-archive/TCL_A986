

#ifndef mainwindow_h
#define mainwindow_h

#include <QtGui>
#include "webpage.h"

class LocationEdit;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(const QString& url = QString());

    void setAddressUrl(const QString& url);
    void addCompleterEntry(const QUrl& url);

    void load(const QString& url);
    void load(const QUrl& url);

    WebPage* page();

protected slots:
    void openFile();
    void changeLocation();

private:
    void buildUI();

    QStringListModel urlModel;
    QStringList urlList;
    LocationEdit* urlEdit;

    WebPage* m_page;
};

#endif
