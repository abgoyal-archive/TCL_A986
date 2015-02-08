
#include "config.h"
#include "KURL.h"
#include "CString.h"
#include "TextEncoding.h"

#include "NotImplemented.h"
#include "qurl.h"

namespace WebCore {

#if QT_VERSION < 0x040500
static const char hexnumbers[] = "0123456789ABCDEF";
static inline char toHex(char c)
{
    return hexnumbers[c & 0xf];
}
#endif

KURL::KURL(const QUrl& url)
{
    *this = KURL(KURL(), url.toEncoded().constData(), UTF8Encoding());
}

KURL::operator QUrl() const
{
#if QT_VERSION < 0x040500
    unsigned length = m_string.length();

    QByteArray ba;
    ba.reserve(length);

    int path = -1;
    int host = m_string.find("://");
    if (host != -1) {
        host += 3;

        path = m_string.find('/', host);
    }

    for (unsigned i = 0; i < length; ++i) {
        const char chr = static_cast<char>(m_string[i]);

        switch (chr) {
            encode:
            case '{':
            case '}':
            case '|':
            case '\\':
            case '^':
            case '`':
                ba.append('%');
                ba.append(toHex((chr & 0xf0) >> 4));
                ba.append(toHex(chr & 0xf));
                break;
            case '[':
            case ']':
                // special case: if this is the host part, don't encode
                // otherwise, encode
                if (host == -1 || (path != -1 && i >= path))
                    goto encode;
                // fall through
            default:
                ba.append(chr);
                break;
        }
    }
#else
    // Qt 4.5 or later
    // No need for special encoding
    QString str = QString::fromRawData(reinterpret_cast<const QChar*>(m_string.characters()), m_string.length());
    QByteArray ba = str.toUtf8();
#endif

    QUrl url = QUrl::fromEncoded(ba);
    return url;
}

String KURL::fileSystemPath() const
{
    if (!isValid() || !protocolIs("file"))
        return String();

    return String(path());
}

}

