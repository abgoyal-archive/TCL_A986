
#include <qwebpluginfactory.h>


class TestPlugin : public QWebPluginFactory
{
public:
    explicit TestPlugin(QObject *parent = 0);
    virtual ~TestPlugin();

    virtual QList<Plugin> plugins() const;

    virtual QObject *create(const QString &mimeType,
                            const QUrl &url,
                            const QStringList &argumentNames,
                            const QStringList &argumentValues) const;

};

