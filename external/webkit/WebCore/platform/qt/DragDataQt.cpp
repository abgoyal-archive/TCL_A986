

#include "config.h"
#include "DragData.h"

#include "ClipboardQt.h"
#include "Document.h"
#include "DocumentFragment.h"
#include "markup.h"

#include <QList>
#include <QMimeData>
#include <QUrl>
#include <QColor>

namespace WebCore {

bool DragData::canSmartReplace() const
{
    return false;
}

bool DragData::containsColor() const
{
    if (!m_platformDragData)
        return false;
    return m_platformDragData->hasColor();
}

bool DragData::containsFiles() const
{
    if (!m_platformDragData)
        return false;
    QList<QUrl> urls = m_platformDragData->urls();
    foreach (const QUrl &url, urls) {
        if (!url.toLocalFile().isEmpty())
            return true;
    }
    return false;
}

void DragData::asFilenames(Vector<String>& result) const
{
    if (!m_platformDragData)
        return;
    QList<QUrl> urls = m_platformDragData->urls();
    foreach (const QUrl &url, urls) {
        QString file = url.toLocalFile();
        if (!file.isEmpty())
            result.append(file);
    }
}

bool DragData::containsPlainText() const
{
    if (!m_platformDragData)
        return false;
    return m_platformDragData->hasText() || m_platformDragData->hasUrls();
}

String DragData::asPlainText() const
{
    if (!m_platformDragData)
        return String();
    String text = m_platformDragData->text();
    if (!text.isEmpty())
        return text;

    // FIXME: Should handle rich text here
    return asURL(0);
}

Color DragData::asColor() const
{
    if (!m_platformDragData)
        return Color();
    return qvariant_cast<QColor>(m_platformDragData->colorData());
}

PassRefPtr<Clipboard> DragData::createClipboard(ClipboardAccessPolicy policy) const
{
    return ClipboardQt::create(policy, m_platformDragData);
}

bool DragData::containsCompatibleContent() const
{
    if (!m_platformDragData)
        return false;
    return containsColor() || containsURL() || m_platformDragData->hasHtml() || m_platformDragData->hasText();
}

bool DragData::containsURL() const
{
    if (!m_platformDragData)
        return false;
    return m_platformDragData->hasUrls();
}

String DragData::asURL(String*) const
{
    if (!m_platformDragData)
        return String();
    QList<QUrl> urls = m_platformDragData->urls();

    if (urls.isEmpty())
        return String();

    return encodeWithURLEscapeSequences(urls.first().toString());
}

PassRefPtr<DocumentFragment> DragData::asFragment(Document* doc) const
{
    if (m_platformDragData && m_platformDragData->hasHtml())
        return createFragmentFromMarkup(doc, m_platformDragData->html(), "", FragmentScriptingNotAllowed);

    return 0;
}

}

