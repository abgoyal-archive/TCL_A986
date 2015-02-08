

#ifndef SVGImage_h
#define SVGImage_h

#if ENABLE(SVG)

#include "Image.h"

namespace WebCore {

    class ImageBuffer;
    class Page;
    class SVGImageChromeClient;
    
    class SVGImage : public Image {
    public:
        static PassRefPtr<SVGImage> create(ImageObserver* observer)
        {
            return adoptRef(new SVGImage(observer));
        }

    private:
        virtual ~SVGImage();

        virtual String filenameExtension() const;

        virtual void setContainerSize(const IntSize&);
        virtual bool usesContainerSize() const;
        virtual bool hasRelativeWidth() const;
        virtual bool hasRelativeHeight() const;

        virtual IntSize size() const;
        
        virtual bool dataChanged(bool allDataReceived);

        // FIXME: SVGImages are underreporting decoded sizes and will be unable
        // to prune because these functions are not implemented yet.
        virtual void destroyDecodedData(bool) { }
        virtual unsigned decodedSize() const { return 0; }

        virtual NativeImagePtr frameAtIndex(size_t) { return 0; }
        
        SVGImage(ImageObserver*);
        virtual void draw(GraphicsContext*, const FloatRect& fromRect, const FloatRect& toRect, ColorSpace styleColorSpace, CompositeOperator);
        
        virtual NativeImagePtr nativeImageForCurrentFrame();
        
        OwnPtr<SVGImageChromeClient> m_chromeClient;
        OwnPtr<Page> m_page;
        OwnPtr<ImageBuffer> m_frameCache;
    };
}

#endif // ENABLE(SVG)

#endif
