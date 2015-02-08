

#include "config.h"
#include "Pasteboard.h"

#include "DocumentFragment.h"
#include "Editor.h"
#include "Frame.h"
#include "Image.h"
#include "markup.h"
#include "RenderImage.h"

#include <qdebug.h>
#include <qclipboard.h>
#include <qmimedata.h>
#include <qapplication.h>
#include <qurl.h>

#define methodDebug() qDebug() << "PasteboardQt: " << __FUNCTION__;

namespace WebCore {

Pasteboard::Pasteboard()
    : m_selectionMode(false)
{
}

Pasteboard* Pasteboard::generalPasteboard()
{
    static Pasteboard* pasteboard = 0;
    if (!pasteboard)
        pasteboard = new Pasteboard();
    return pasteboard;
}

void Pasteboard::writeSelection(Range* selectedRange, bool, Frame* frame)
{
    QMimeData* md = new QMimeData;
    QString text = frame->selectedText();
    text.replace(QChar(0xa0), QLatin1Char(' '));
    md->setText(text);

    QString html = QLatin1String("<html><head><meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\" /></head><body>");
    html += createMarkup(selectedRange, 0, AnnotateForInterchange);
    html += QLatin1String("</body></html>");
    md->setHtml(html);

#ifndef QT_NO_CLIPBOARD
    QApplication::clipboard()->setMimeData(md, m_selectionMode ?
            QClipboard::Selection : QClipboard::Clipboard);
#endif
}

bool Pasteboard::canSmartReplace()
{
    return false;
}

String Pasteboard::plainText(Frame*)
{
#ifndef QT_NO_CLIPBOARD
    return QApplication::clipboard()->text(m_selectionMode ?
            QClipboard::Selection : QClipboard::Clipboard);
#else
    return String();
#endif
}

PassRefPtr<DocumentFragment> Pasteboard::documentFragment(Frame* frame, PassRefPtr<Range> context,
                                                          bool allowPlainText, bool& chosePlainText)
{
#ifndef QT_NO_CLIPBOARD
    const QMimeData* mimeData = QApplication::clipboard()->mimeData(
            m_selectionMode ? QClipboard::Selection : QClipboard::Clipboard);

    chosePlainText = false;

    if (mimeData->hasHtml()) {
        QString html = mimeData->html();
        if (!html.isEmpty()) {
            RefPtr<DocumentFragment> fragment = createFragmentFromMarkup(frame->document(), html, "", FragmentScriptingNotAllowed);
            if (fragment)
                return fragment.release();
        }
    }

    if (allowPlainText && mimeData->hasText()) {
        chosePlainText = true;
        RefPtr<DocumentFragment> fragment = createFragmentFromText(context.get(), mimeData->text());
        if (fragment)
            return fragment.release();
    }
#endif
    return 0;
}

void Pasteboard::writePlainText(const String& text)
{
#ifndef QT_NO_CLIPBOARD
    QMimeData* md = new QMimeData;
    QString qtext = text;
    qtext.replace(QChar(0xa0), QLatin1Char(' '));
    md->setText(qtext);
    QApplication::clipboard()->setMimeData(md, m_selectionMode ?
            QClipboard::Selection : QClipboard::Clipboard);
#endif
}

void Pasteboard::writeURL(const KURL& _url, const String&, Frame*)
{
    ASSERT(!_url.isEmpty());

#ifndef QT_NO_CLIPBOARD
    QMimeData* md = new QMimeData;
    QString url = _url.string();
    md->setText(url);
    md->setUrls(QList<QUrl>() << QUrl(url));
    QApplication::clipboard()->setMimeData(md, m_selectionMode ?
            QClipboard::Selection : QClipboard::Clipboard);
#endif
}

void Pasteboard::writeImage(Node* node, const KURL&, const String&)
{
    ASSERT(node && node->renderer() && node->renderer()->isImage());

#ifndef QT_NO_CLIPBOARD
    CachedImage* cachedImage = toRenderImage(node->renderer())->cachedImage();
    ASSERT(cachedImage);

    Image* image = cachedImage->image();
    ASSERT(image);

    QPixmap* pixmap = image->nativeImageForCurrentFrame();
    ASSERT(pixmap);

    QApplication::clipboard()->setPixmap(*pixmap, QClipboard::Clipboard);
#endif
}

void Pasteboard::clear()
{
}

bool Pasteboard::isSelectionMode() const
{
    return m_selectionMode;
}

void Pasteboard::setSelectionMode(bool selectionMode)
{
    m_selectionMode = selectionMode;
}

}
