

#include "Image.h"

#include "FloatRect.h"
#include "GraphicsTypes.h"

#if PLATFORM(CG)

#include <ApplicationServices/ApplicationServices.h>

namespace WebCore {

    class GraphicsContext;

    class PDFDocumentImage : public Image {
    public:
        static PassRefPtr<PDFDocumentImage> create()
        {
            return adoptRef(new PDFDocumentImage);
        }

    private:
        virtual ~PDFDocumentImage();

        virtual bool hasSingleSecurityOrigin() const { return true; }

        virtual bool dataChanged(bool allDataReceived);

        // FIXME: PDF Images are underreporting decoded sizes and will be unable
        // to prune because these functions are not implemented yet.
        virtual void destroyDecodedData(bool /*destroyAll*/ = true) { }
        virtual unsigned decodedSize() const { return 0; }

        virtual IntSize size() const;

        PDFDocumentImage();
        virtual void draw(GraphicsContext*, const FloatRect& dstRect, const FloatRect& srcRect, ColorSpace styleColorSpace, CompositeOperator);
        
        void setCurrentPage(int);
        int pageCount() const;
        void adjustCTM(GraphicsContext*) const;

        CGPDFDocumentRef m_document;
        FloatRect m_mediaBox;
        FloatRect m_cropBox;
        float m_rotation;
        int m_currentPage;
    };

}

#endif // PLATFORM(CG)
