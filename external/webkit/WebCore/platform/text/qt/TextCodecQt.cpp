

#include "config.h"
#include "TextCodecQt.h"
#include "PlatformString.h"
#include "CString.h"
#include <qset.h>
// #include <QDebug>

namespace WebCore {

static QSet<QByteArray> *unique_names = 0;

static const char *getAtomicName(const QByteArray &name)
{
    if (!unique_names)
        unique_names = new QSet<QByteArray>;

    unique_names->insert(name);
    return unique_names->find(name)->constData();
}

void TextCodecQt::registerEncodingNames(EncodingNameRegistrar registrar)
{
    QList<int> mibs = QTextCodec::availableMibs();
//     qDebug() << ">>>>>>>>> registerEncodingNames";

    for (int i = 0; i < mibs.size(); ++i) {
        QTextCodec *c = QTextCodec::codecForMib(mibs.at(i));
        const char *name = getAtomicName(c->name());
        registrar(name, name);
//         qDebug() << "    " << name << name;
        QList<QByteArray> aliases = c->aliases();
        for (int i = 0; i < aliases.size(); ++i) {
            const char *a = getAtomicName(aliases.at(i));
//             qDebug() << "     (a) " << a << name;
            registrar(a, name);
        }
    }
}

static PassOwnPtr<TextCodec> newTextCodecQt(const TextEncoding& encoding, const void*)
{
    return new TextCodecQt(encoding);
}

void TextCodecQt::registerCodecs(TextCodecRegistrar registrar)
{
    QList<int> mibs = QTextCodec::availableMibs();
//     qDebug() << ">>>>>>>>> registerCodecs";

    for (int i = 0; i < mibs.size(); ++i) {
        QTextCodec *c = QTextCodec::codecForMib(mibs.at(i));
        const char *name = getAtomicName(c->name());
//         qDebug() << "    " << name;
        registrar(name, newTextCodecQt, 0);
    }
}

TextCodecQt::TextCodecQt(const TextEncoding& encoding)
    : m_encoding(encoding)
{
    m_codec = QTextCodec::codecForName(m_encoding.name());
}

TextCodecQt::~TextCodecQt()
{
}


String TextCodecQt::decode(const char* bytes, size_t length, bool flush, bool /*stopOnError*/, bool& sawError)
{
    // We chop input buffer to smaller buffers to avoid excessive memory consumption
    // when the input buffer is big.  This helps reduce peak memory consumption in
    // mobile devices where system RAM is limited.
#if OS(SYMBIAN)
    static const int MaxInputChunkSize = 32 * 1024;
#else
    static const int MaxInputChunkSize = 1024 * 1024;
#endif
    const char* buf = bytes;
    const char* end = buf + length;
    String unicode(""); // a non-null string is expected

    while (buf < end) {
        int size = end - buf;
        size = qMin(size, MaxInputChunkSize);
        QString decoded = m_codec->toUnicode(buf, size, &m_state);
        unicode.append(decoded);
        buf += size;
    }

    sawError = m_state.invalidChars != 0;

    if (flush) {
        m_state.flags = QTextCodec::DefaultConversion;
        m_state.remainingChars = 0;
        m_state.invalidChars = 0;
    }

    return unicode;
}

CString TextCodecQt::encode(const UChar* characters, size_t length, UnencodableHandling)
{
    if (!length)
        return "";

    // FIXME: do something sensible with UnencodableHandling

    QByteArray ba = m_codec->fromUnicode(reinterpret_cast<const QChar*>(characters), length, 0);
    return CString(ba.constData(), ba.length());
}


} // namespace WebCore
