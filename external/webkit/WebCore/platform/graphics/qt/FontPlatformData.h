
#ifndef FontPlatformData_h
#define FontPlatformData_h

#include "FontDescription.h"
#include <QFont>
#include <QHash>

namespace WebCore {

class String;
class FontPlatformDataPrivate : public Noncopyable {
public:
    FontPlatformDataPrivate()
        : refCount(1)
        , size(font.pointSizeF())
        , bold(font.bold())
        , oblique(false)
    {}
    FontPlatformDataPrivate(const float size, const bool bold, const bool oblique)
        : refCount(1)
        , size(size)
        , bold(bold)
        , oblique(oblique)
    {}
    FontPlatformDataPrivate(const QFont& font)
        : refCount(1)
        , font(font)
        , size(font.pointSizeF())
        , bold(font.bold())
        , oblique(false)
    {}
    unsigned refCount;
    QFont font;
    float size;
    bool bold : 1;
    bool oblique : 1;
};



class FontPlatformData : public FastAllocBase {
public:
    FontPlatformData(float size, bool bold, bool oblique);
    FontPlatformData(const FontPlatformData &);
    FontPlatformData(const FontDescription&, const AtomicString& familyName, int wordSpacing = 0, int letterSpacing = 0);
    FontPlatformData(const QFont& font)
        : m_data(new FontPlatformDataPrivate(font))
    {}
    FontPlatformData(WTF::HashTableDeletedValueType)
        : m_data(reinterpret_cast<FontPlatformDataPrivate*>(-1))
    {}

    ~FontPlatformData();

    FontPlatformData& operator=(const FontPlatformData&);
    bool operator==(const FontPlatformData&) const;

    bool isHashTableDeletedValue() const
    {
        return m_data == reinterpret_cast<FontPlatformDataPrivate*>(-1);
    }

    static inline QFont::Weight toQFontWeight(FontWeight fontWeight)
    {
        switch (fontWeight) {
        case FontWeight100:
        case FontWeight200:
            return QFont::Light;  // QFont::Light == Weight of 25
        case FontWeight600:
            return QFont::DemiBold;  // QFont::DemiBold == Weight of 63
        case FontWeight700:
        case FontWeight800:
            return QFont::Bold;  // QFont::Bold == Weight of 75
        case FontWeight900:
            return QFont::Black;  // QFont::Black == Weight of 87
        case FontWeight300:
        case FontWeight400:
        case FontWeight500:
        default:
            return QFont::Normal;  // QFont::Normal == Weight of 50
        }
    }

    QFont font() const
    {
        Q_ASSERT(m_data != reinterpret_cast<FontPlatformDataPrivate*>(-1));
        if (m_data)
            return m_data->font;
        return QFont();
    }
    float size() const
    {
        Q_ASSERT(m_data != reinterpret_cast<FontPlatformDataPrivate*>(-1));
        if (m_data)
            return m_data->size;
        return 0.0f;
    }
    QString family() const
    {
        Q_ASSERT(m_data != reinterpret_cast<FontPlatformDataPrivate*>(-1));
        if (m_data)
            return m_data->font.family();
        return QString();
    }
    bool bold() const
    {
        Q_ASSERT(m_data != reinterpret_cast<FontPlatformDataPrivate*>(-1));
        if (m_data)
            return m_data->bold;
        return false;
    }
    bool italic() const
    {
        Q_ASSERT(m_data != reinterpret_cast<FontPlatformDataPrivate*>(-1));
        if (m_data)
            return m_data->font.italic();
        return false;
    }
    bool smallCaps() const
    {
        Q_ASSERT(m_data != reinterpret_cast<FontPlatformDataPrivate*>(-1));
        if (m_data)
            return m_data->font.capitalization() == QFont::SmallCaps;
        return false;
    }
    int pixelSize() const
    {
        Q_ASSERT(m_data != reinterpret_cast<FontPlatformDataPrivate*>(-1));
        if (m_data)
            return m_data->font.pixelSize();
        return 0;
    }
    unsigned hash() const;

#ifndef NDEBUG
    String description() const;
#endif
private:
    FontPlatformDataPrivate* m_data;
};

} // namespace WebCore

#endif // FontPlatformData_h
