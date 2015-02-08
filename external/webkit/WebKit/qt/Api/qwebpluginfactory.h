

#ifndef QWEBPLUGINFACTORY_H
#define QWEBPLUGINFACTORY_H

#include "qwebkitglobal.h"

#include <QtCore/qobject.h>
#include <QtCore/qstringlist.h>

QT_BEGIN_NAMESPACE
class QUrl;
class QString;
QT_END_NAMESPACE
class QWebPluginFactoryPrivate;

class QWEBKIT_EXPORT QWebPluginFactory : public QObject {
    Q_OBJECT
public:
    struct QWEBKIT_EXPORT MimeType {
        QString name;
        QString description;
        QStringList fileExtensions;
        bool operator==(const MimeType& other) const;
        inline bool operator!=(const MimeType& other) const { return !operator==(other); }
    };

    struct Plugin {
        QString name;
        QString description;
        QList<MimeType> mimeTypes;
    };

    explicit QWebPluginFactory(QObject* parent = 0);
    virtual ~QWebPluginFactory();

    virtual QList<Plugin> plugins() const = 0;
    virtual void refreshPlugins();

    virtual QObject *create(const QString& mimeType,
                            const QUrl&,
                            const QStringList& argumentNames,
                            const QStringList& argumentValues) const = 0;

    enum Extension {
    };
    class ExtensionOption
    {};
    class ExtensionReturn
    {};
    virtual bool extension(Extension extension, const ExtensionOption* option = 0, ExtensionReturn* output = 0);
    virtual bool supportsExtension(Extension extension) const;

private:
    QWebPluginFactoryPrivate* d;
};

#endif
