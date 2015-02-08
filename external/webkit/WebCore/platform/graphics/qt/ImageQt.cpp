

#include "config.h"
#include "Image.h"

#include "AffineTransform.h"
#include "ImageObserver.h"
#include "BitmapImage.h"
#include "FloatRect.h"
#include "PlatformString.h"
#include "GraphicsContext.h"
#include "StillImageQt.h"
#include "qwebsettings.h"

#include <QPixmap>
#include <QPainter>
#include <QImage>
#include <QImageReader>
#include <QTransform>

#include <QDebug>

#include <math.h>

// This function loads resources into WebKit
static QPixmap loadResourcePixmap(const char *name)
{
    QPixmap pixmap;
    if (qstrcmp(name, "missingImage") == 0)
        pixmap = QWebSettings::webGraphic(QWebSettings::MissingImageGraphic);
    else if (qstrcmp(name, "nullPlugin") == 0)
        pixmap = QWebSettings::webGraphic(QWebSettings::MissingPluginGraphic);
    else if (qstrcmp(name, "urlIcon") == 0)
        pixmap = QWebSettings::webGraphic(QWebSettings::DefaultFrameIconGraphic);
    else if (qstrcmp(name, "textAreaResizeCorner") == 0)
        pixmap = QWebSettings::webGraphic(QWebSettings::TextAreaSizeGripCornerGraphic);
    else if (qstrcmp(name, "deleteButton") == 0)
        pixmap = QWebSettings::webGraphic(QWebSettings::DeleteButtonGraphic);

    return pixmap;
}

namespace WebCore {

bool FrameData::clear(bool clearMetadata)
{
    if (clearMetadata)
        m_haveMetadata = false;

    if (m_frame) {
        delete m_frame;
        m_frame = 0;
        return true;
    }
    return false;
}


// ================================================
// Image Class
// ================================================

PassRefPtr<Image> Image::loadPlatformResource(const char* name)
{
    return StillImage::create(loadResourcePixmap(name));
}

void Image::drawPattern(GraphicsContext* ctxt, const FloatRect& tileRect, const AffineTransform& patternTransform,
                        const FloatPoint& phase, ColorSpace, CompositeOperator op, const FloatRect& destRect)
{
    QPixmap* framePixmap = nativeImageForCurrentFrame();
    if (!framePixmap) // If it's too early we won't have an image yet.
        return;

    QPixmap pixmap = *framePixmap;
    QRect tr = QRectF(tileRect).toRect();
    if (tr.x() || tr.y() || tr.width() != pixmap.width() || tr.height() != pixmap.height())
        pixmap = pixmap.copy(tr);

    QBrush b(pixmap);
    b.setTransform(patternTransform);
    ctxt->save();
    ctxt->setCompositeOperation(op);
    QPainter* p = ctxt->platformContext();
    if (!pixmap.hasAlpha() && p->compositionMode() == QPainter::CompositionMode_SourceOver)
        p->setCompositionMode(QPainter::CompositionMode_Source);
    p->setBrushOrigin(phase);
    p->fillRect(destRect, b);
    ctxt->restore();

    if (imageObserver())
        imageObserver()->didDraw(this);
}

BitmapImage::BitmapImage(QPixmap* pixmap, ImageObserver* observer)
    : Image(observer)
    , m_currentFrame(0)
    , m_frames(0)
    , m_frameTimer(0)
    , m_repetitionCount(cAnimationNone)
    , m_repetitionCountStatus(Unknown)
    , m_repetitionsComplete(0)
    , m_isSolidColor(false)
    , m_checkedForSolidColor(false)
    , m_animationFinished(true)
    , m_allDataReceived(true)
    , m_haveSize(true)
    , m_sizeAvailable(true)
    , m_decodedSize(0)
    , m_haveFrameCount(true)
    , m_frameCount(1)
{
    initPlatformData();

    int width = pixmap->width();
    int height = pixmap->height();
    m_decodedSize = width * height * 4;
    m_size = IntSize(width, height);

    m_frames.grow(1);
    m_frames[0].m_frame = pixmap;
    m_frames[0].m_hasAlpha = pixmap->hasAlpha();
    m_frames[0].m_haveMetadata = true;
    checkForSolidColor();
}

void BitmapImage::initPlatformData()
{
}

void BitmapImage::invalidatePlatformData()
{
}

// Drawing Routines
void BitmapImage::draw(GraphicsContext* ctxt, const FloatRect& dst,
                       const FloatRect& src, ColorSpace styleColorSpace, CompositeOperator op)
{
    startAnimation();

    QPixmap* image = nativeImageForCurrentFrame();
    if (!image)
        return;

    if (mayFillWithSolidColor()) {
        fillWithSolidColor(ctxt, dst, solidColor(), styleColorSpace, op);
        return;
    }

    IntSize selfSize = size();

    ctxt->save();

    // Set the compositing operation.
    ctxt->setCompositeOperation(op);

    QPainter* painter(ctxt->platformContext());

    if (!image->hasAlpha() && painter->compositionMode() == QPainter::CompositionMode_SourceOver)
        painter->setCompositionMode(QPainter::CompositionMode_Source);

    // Test using example site at
    // http://www.meyerweb.com/eric/css/edge/complexspiral/demo.html
    painter->drawPixmap(dst, *image, src);

    ctxt->restore();

    if (imageObserver())
        imageObserver()->didDraw(this);
}

void BitmapImage::checkForSolidColor()
{
    m_isSolidColor = false;
    m_checkedForSolidColor = true;

    if (frameCount() > 1)
        return;

    QPixmap* framePixmap = frameAtIndex(0);
    if (!framePixmap || framePixmap->width() != 1 || framePixmap->height() != 1)
        return;

    m_isSolidColor = true;
    m_solidColor = QColor::fromRgba(framePixmap->toImage().pixel(0, 0));
}

#if OS(WINDOWS)
PassRefPtr<BitmapImage> BitmapImage::create(HBITMAP hBitmap)
{
    return BitmapImage::create(new QPixmap(QPixmap::fromWinHBITMAP(hBitmap)));
}
#endif

}


// vim: ts=4 sw=4 et
