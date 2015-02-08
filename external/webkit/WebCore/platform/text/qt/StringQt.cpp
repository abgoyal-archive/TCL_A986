

#include "config.h"

#include "PlatformString.h"

#include <QString>

namespace WebCore {

// String conversions
String::String(const QString& qstr)
{
    if (qstr.isNull())
        return;
    m_impl = StringImpl::create(reinterpret_cast<const UChar*>(qstr.constData()), qstr.length());
}

String::String(const QStringRef& ref)
{
    if (!ref.string())
        return;
    m_impl = StringImpl::create(reinterpret_cast<const UChar*>(ref.unicode()), ref.length());
}

String::operator QString() const
{
    return QString(reinterpret_cast<const QChar*>(characters()), length());
}

QDataStream& operator<<(QDataStream& stream, const String& str)
{
    // could be faster
    stream << QString(str);
    return stream;
}

QDataStream& operator>>(QDataStream& stream, String& str)
{
    // mabe not the fastest way, but really easy
    QString tmp;
    stream >> tmp;
    str = tmp;
    return stream;
}

}

// vim: ts=4 sw=4 et
