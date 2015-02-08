

#include "config.h"
#include "qwebplugindatabase_p.h"

#include "PluginDatabase.h"
#include "PluginPackage.h"

using namespace WebCore;



QWebPluginInfo::QWebPluginInfo()
    : m_package(0)
{
}

QWebPluginInfo::QWebPluginInfo(PluginPackage* package)
    : m_package(package)
{
    if (m_package)
        m_package->ref();
}

QWebPluginInfo::QWebPluginInfo(const QWebPluginInfo& other)
    : m_package(other.m_package)
{
    if (m_package)
        m_package->ref();
}

QWebPluginInfo::~QWebPluginInfo()
{
    if (m_package)
        m_package->deref();
}

QString QWebPluginInfo::name() const
{
    if (!m_package)
        return QString();
    return m_package->name();
}

QString QWebPluginInfo::description() const
{
    if (!m_package)
        return QString();
    return m_package->description();
}

QList<QWebPluginInfo::MimeType> QWebPluginInfo::mimeTypes() const
{
    if (m_package && m_mimeTypes.isEmpty()) {
        const MIMEToDescriptionsMap& mimeToDescriptions = m_package->mimeToDescriptions();
        MIMEToDescriptionsMap::const_iterator end = mimeToDescriptions.end();

        for (MIMEToDescriptionsMap::const_iterator it = mimeToDescriptions.begin(); it != end; ++it) {
            MimeType mimeType;
            mimeType.name = it->first;
            mimeType.description = it->second;

            QStringList fileExtensions;
            Vector<String> extensions = m_package->mimeToExtensions().get(mimeType.name);

            for (unsigned i = 0; i < extensions.size(); ++i)
                fileExtensions.append(extensions[i]);

            mimeType.fileExtensions = fileExtensions;
            m_mimeTypes.append(mimeType);
        }
    }

    return m_mimeTypes;
}

bool QWebPluginInfo::supportsMimeType(const QString& mimeType) const
{
    if (!m_package)
        return false;
    return m_package->mimeToDescriptions().contains(mimeType);
}

QString QWebPluginInfo::path() const
{
    if (!m_package)
        return QString();
    return m_package->path();
}

bool QWebPluginInfo::isNull() const
{
    return !m_package;
}

void QWebPluginInfo::setEnabled(bool enabled)
{
    if (!m_package)
        return;
    m_package->setEnabled(enabled);
}

bool QWebPluginInfo::isEnabled() const
{
    if (!m_package)
        return false;
    return m_package->isEnabled();
}

bool QWebPluginInfo::operator==(const QWebPluginInfo& other) const
{
    return m_package == other.m_package;
}

bool QWebPluginInfo::operator!=(const QWebPluginInfo& other) const
{
    return m_package != other.m_package;
}

QWebPluginInfo &QWebPluginInfo::operator=(const QWebPluginInfo& other)
{
    if (this == &other)
        return *this;

    if (m_package)
        m_package->deref();
    m_package = other.m_package;
    if (m_package)
        m_package->ref();
    m_mimeTypes = other.m_mimeTypes;

    return *this;
}


QWebPluginDatabase::QWebPluginDatabase(QObject* parent)
    : QObject(parent)
    , m_database(PluginDatabase::installedPlugins())
{
}

QWebPluginDatabase::~QWebPluginDatabase()
{
}

QList<QWebPluginInfo> QWebPluginDatabase::plugins() const
{
    QList<QWebPluginInfo> qwebplugins;
    const Vector<PluginPackage*>& plugins = m_database->plugins();

    for (unsigned int i = 0; i < plugins.size(); ++i) {
        PluginPackage* plugin = plugins[i];
        qwebplugins.append(QWebPluginInfo(plugin));
    }

    return qwebplugins;
}

QStringList QWebPluginDatabase::defaultSearchPaths()
{
    QStringList paths;

    const Vector<String>& directories = PluginDatabase::defaultPluginDirectories();
    for (unsigned int i = 0; i < directories.size(); ++i)
        paths.append(directories[i]);

    return paths;
}

QStringList QWebPluginDatabase::searchPaths() const
{
    QStringList paths;

    const Vector<String>& directories = m_database->pluginDirectories();
    for (unsigned int i = 0; i < directories.size(); ++i)
        paths.append(directories[i]);

    return paths;
}

void QWebPluginDatabase::setSearchPaths(const QStringList& paths)
{
    Vector<String> directories;

    for (int i = 0; i < paths.count(); ++i)
        directories.append(paths.at(i));

    m_database->setPluginDirectories(directories);
    // PluginDatabase::setPluginDirectories() does not refresh the database.
    m_database->refresh();
}

void QWebPluginDatabase::addSearchPath(const QString& path)
{
    m_database->addExtraPluginDirectory(path);
    // PluginDatabase::addExtraPluginDirectory() does refresh the database.
}

void QWebPluginDatabase::refresh()
{
    m_database->refresh();
}

QWebPluginInfo QWebPluginDatabase::pluginForMimeType(const QString& mimeType)
{
    return QWebPluginInfo(m_database->pluginForMIMEType(mimeType));
}

void QWebPluginDatabase::setPreferredPluginForMimeType(const QString& mimeType, const QWebPluginInfo& plugin)
{
    m_database->setPreferredPluginForMIMEType(mimeType, plugin.m_package);
}
