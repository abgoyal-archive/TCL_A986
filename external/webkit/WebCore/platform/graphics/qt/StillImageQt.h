

#ifndef StillImageQt_h
#define StillImageQt_h

#include "Image.h"

namespace WebCore {

    class StillImage : public Image {
    public:
        static PassRefPtr<StillImage> create(const QPixmap& pixmap)
        {
            return adoptRef(new StillImage(pixmap));
        }

        // FIXME: StillImages are underreporting decoded sizes and will be unable
        // to prune because these functions are not implemented yet.
        virtual void destroyDecodedData(bool destroyAll = true) { Q_UNUSED(destroyAll); }
        virtual unsigned decodedSize() const { return 0; }

        virtual IntSize size() const;
        virtual NativeImagePtr nativeImageForCurrentFrame();
        virtual void draw(GraphicsContext*, const FloatRect& dstRect, const FloatRect& srcRect, ColorSpace styleColorSpace, CompositeOperator);

    private:
        StillImage(const QPixmap& pixmap);
        
        QPixmap m_pixmap;
    };

}

#endif
