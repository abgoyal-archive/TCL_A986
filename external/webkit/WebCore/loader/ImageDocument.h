

#ifndef ImageDocument_h
#define ImageDocument_h

#include "HTMLDocument.h"

namespace WebCore {
    
class ImageDocumentElement;

class ImageDocument : public HTMLDocument {
public:
    static PassRefPtr<ImageDocument> create(Frame* frame)
    {
        return adoptRef(new ImageDocument(frame));
    }

    CachedImage* cachedImage();
    ImageDocumentElement* imageElement() const { return m_imageElement; }
    void disconnectImageElement() { m_imageElement = 0; }
    
    void windowSizeChanged();
    void imageChanged();
    void imageClicked(int x, int y);

private:
    ImageDocument(Frame*);

    virtual Tokenizer* createTokenizer();
    virtual bool isImageDocument() const { return true; }
    
    void createDocumentStructure();
    void resizeImageToFit();
    void restoreImageSize();
    bool imageFitsInWindow() const;
    bool shouldShrinkToFit() const;
    float scale() const;
    
    ImageDocumentElement* m_imageElement;
    
    // Whether enough of the image has been loaded to determine its size
    bool m_imageSizeIsKnown;
    
    // Whether the image is shrunk to fit or not
    bool m_didShrinkImage;
    
    // Whether the image should be shrunk or not
    bool m_shouldShrinkImage;
};
    
}

#endif // ImageDocument_h
