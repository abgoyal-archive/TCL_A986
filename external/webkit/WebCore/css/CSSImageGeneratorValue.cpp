

#include "config.h"
#include "CSSImageGeneratorValue.h"

#include "Image.h"
#include "IntSize.h"
#include "IntSizeHash.h"
#include "PlatformString.h"
#include "RenderObject.h"
#include "StyleGeneratedImage.h"

namespace WebCore {

CSSImageGeneratorValue::CSSImageGeneratorValue()
: m_accessedImage(false)
{
}

CSSImageGeneratorValue::~CSSImageGeneratorValue()
{
}

void CSSImageGeneratorValue::addClient(RenderObject* renderer, const IntSize& size)
{
    ref();
    if (!size.isEmpty())
        m_sizes.add(size);
    
    RenderObjectSizeCountMap::iterator it = m_clients.find(renderer);
    if (it == m_clients.end())
        m_clients.add(renderer, SizeCountPair(size, 1));
    else {
        SizeCountPair& sizeCount = it->second;
        ++sizeCount.second;
    }
}

void CSSImageGeneratorValue::removeClient(RenderObject* renderer)
{
    RenderObjectSizeCountMap::iterator it = m_clients.find(renderer);
    ASSERT(it != m_clients.end());

    SizeCountPair& sizeCount = it->second;
    IntSize size = sizeCount.first;
    if (!size.isEmpty()) {
        m_sizes.remove(size);
        if (!m_sizes.contains(size))
            m_images.remove(size);
    }
    
    if (!--sizeCount.second)
        m_clients.remove(renderer);

    deref();
}

Image* CSSImageGeneratorValue::getImage(RenderObject* renderer, const IntSize& size)
{
    RenderObjectSizeCountMap::iterator it = m_clients.find(renderer);
    ASSERT(it != m_clients.end());

    SizeCountPair& sizeCount = it->second;
    IntSize oldSize = sizeCount.first;
    if (oldSize != size) {
        removeClient(renderer);
        addClient(renderer, size);
    }
    
    // Don't generate an image for empty sizes.
    if (size.isEmpty())
        return 0;
    
    // Look up the image in our cache.
    return m_images.get(size).get();
}

void CSSImageGeneratorValue::putImage(const IntSize& size, PassRefPtr<Image> image)
{
    m_images.add(size, image);
}

StyleGeneratedImage* CSSImageGeneratorValue::generatedImage()
{
    if (!m_accessedImage) {
        m_accessedImage = true;
        m_image = StyleGeneratedImage::create(this, isFixedSize());
    }
    return m_image.get();
}

} // namespace WebCore
