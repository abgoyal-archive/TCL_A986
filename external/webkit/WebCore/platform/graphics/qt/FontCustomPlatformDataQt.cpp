
#include "config.h"
#include "FontCustomPlatformData.h"

#include "FontPlatformData.h"
#include "SharedBuffer.h"
#include <QFontDatabase>
#include <QStringList>

namespace WebCore {

FontCustomPlatformData::~FontCustomPlatformData()
{
    QFontDatabase::removeApplicationFont(m_handle);
}

FontPlatformData FontCustomPlatformData::fontPlatformData(int size, bool bold, bool italic, FontRenderingMode)
{
    QFont font;
    font.setFamily(QFontDatabase::applicationFontFamilies(m_handle)[0]);
    font.setPixelSize(size);
    if (bold)
        font.setWeight(QFont::Bold);
    font.setItalic(italic);

    return FontPlatformData(font);
}

FontCustomPlatformData* createFontCustomPlatformData(SharedBuffer* buffer)
{
    ASSERT_ARG(buffer, buffer);

    int id = QFontDatabase::addApplicationFontFromData(QByteArray(buffer->data(), buffer->size()));
    if (id == -1)
        return 0;

    Q_ASSERT(QFontDatabase::applicationFontFamilies(id).size() > 0);

    FontCustomPlatformData *data = new FontCustomPlatformData;
    data->m_handle = id;
    return data;
}

}

