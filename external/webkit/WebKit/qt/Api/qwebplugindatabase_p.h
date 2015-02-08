

#ifndef QWEBPLUGINDATABASE_H
#define QWEBPLUGINDATABASE_H

#include "qwebkitglobal.h"
#include "qwebpluginfactory.h"

#include <QtCore/qobject.h>
#include <QtCore/qstringlist.h>

namespace WebCore {
    class PluginDatabase;
    class PluginPackage;
}

class QWebPluginInfoPrivate;
class QWEBKIT_EXPORT QWebPluginInfo {
public:
    QWebPluginInfo();
    QWebPluginInfo(const QWebPluginInfo& other);
    QWebPluginInfo &operator=(const QWebPluginInfo& other);
    ~QWebPluginInfo();

private:
    QWebPluginInfo(WebCore::PluginPackage* package);

public:
    typedef QWebPluginFactory::MimeType MimeType;

    QString name() const;
    QString description() const;
    QList<MimeType> mimeTypes() const;
    bool supportsMimeType(const QString& mimeType) const;
    QString path() const;

    bool isNull() const;

    void setEnabled(bool enabled);
    bool isEnabled() const;

    bool operator==(const QWebPluginInfo& other) const;
    bool operator!=(const QWebPluginInfo& other) const;

    friend class QWebPluginDatabase;

private:
    QWebPluginInfoPrivate* d;
    WebCore::PluginPackage* m_package;
    mutable QList<MimeType> m_mimeTypes;
};

class QWebPluginDatabasePrivate;
class QWEBKIT_EXPORT QWebPluginDatabase : public QObject {
    Q_OBJECT

private:
    QWebPluginDatabase(QObject* parent = 0);
    ~QWebPluginDatabase();

public:
    QList<QWebPluginInfo> plugins() const;

    static QStringList defaultSearchPaths();
    QStringList searchPaths() const;
    void setSearchPaths(const QStringList& paths);
    void addSearchPath(const QString& path);

    void refresh();

    QWebPluginInfo pluginForMimeType(const QString& mimeType);
    void setPreferredPluginForMimeType(const QString& mimeType, const QWebPluginInfo& plugin);

    friend class QWebSettings;

private:
    QWebPluginDatabasePrivate* d;
    WebCore::PluginDatabase* m_database;
};

#endif // QWEBPLUGINDATABASE_H
