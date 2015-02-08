

#ifndef utils_h
#define utils_h

#include <wtf/AlwaysInline.h>

#include <QtCore>

// options handling
QString takeOptionValue(QStringList* arguments, int index);
QString formatKeys(QList<QString> keys);
QList<QString> enumToKeys(const QMetaObject o, const QString& name, const QString& strip);

NO_RETURN void appQuit(int status, const QString& msg = QString());

QUrl urlFromUserInput(const QString& input);

#endif
