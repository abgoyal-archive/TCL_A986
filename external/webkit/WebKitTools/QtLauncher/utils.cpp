

#include "utils.h"


QString takeOptionValue(QStringList* arguments, int index)
{
    QString result;

    if (++index < arguments->count() && !arguments->at(index).startsWith("-"))
        result = arguments->takeAt(index);

    return result;
}

QString formatKeys(QList<QString> keys)
{
    QString result;
    for (int i = 0; i < keys.count() - 1; i++)
        result.append(keys.at(i) + "|");
    result.append(keys.last());
    return result;
}

QList<QString> enumToKeys(const QMetaObject o, const QString& name, const QString& strip)
{
    QList<QString> list;

    int enumIndex = o.indexOfEnumerator(name.toLatin1().data());
    QMetaEnum enumerator = o.enumerator(enumIndex);

    if (enumerator.isValid()) {
        for (int i = 0; i < enumerator.keyCount(); i++) {
            QString key(enumerator.valueToKey(i));
            list.append(key.remove(strip));
        }
    }

    return list;
}

void appQuit(int exitCode, const QString& msg)
{
    if (!msg.isEmpty()) {
        if (exitCode > 0)
            qDebug("ERROR: %s", msg.toLatin1().data());
        else
            qDebug() << msg;
    }
    exit(exitCode);
}

QUrl urlFromUserInput(const QString& string)
{
    QString input(string);
    QFileInfo fi(input);
    if (fi.exists() && fi.isRelative())
        input = fi.absoluteFilePath();

#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
    return QUrl::fromUserInput(input);
#else
    return QUrl(input);
#endif
}


