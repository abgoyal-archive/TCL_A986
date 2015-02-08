
#include "testplugin.h"

TestPlugin::TestPlugin(QObject *parent)
    : QWebPluginFactory(parent)
{
}

TestPlugin::~TestPlugin()
{
}

QList<QWebPluginFactory::Plugin> TestPlugin::plugins() const
{
    QWebPluginFactory::Plugin plugin;

    plugin.name = "testplugin";
    plugin.description = "testdescription";
    MimeType mimeType;
    mimeType.name = "testtype";
    mimeType.fileExtensions.append("testsuffixes");
    plugin.mimeTypes.append(mimeType);

    plugin.name = "testplugin2";
    plugin.description = "testdescription2";
    mimeType.name = "testtype2";
    mimeType.fileExtensions.append("testsuffixes2");
    mimeType.fileExtensions.append("testsuffixes3");
    plugin.mimeTypes.append(mimeType);

    return QList<QWebPluginFactory::Plugin>() << plugin;
}

QObject *TestPlugin::create(const QString&,
                            const QUrl&,
                            const QStringList&,
                            const QStringList&) const
{
    return 0;
}

