

#include "config.h"
#include "HTMLPlugInImageElement.h"

#include "Frame.h"
#include "FrameLoader.h"
#include "FrameLoaderClient.h"
#include "HTMLImageLoader.h"
#include "Image.h"

namespace WebCore {

HTMLPlugInImageElement::HTMLPlugInImageElement(const QualifiedName& tagName, Document* document)
    : HTMLPlugInElement(tagName, document)
{
}

HTMLPlugInImageElement::~HTMLPlugInImageElement()
{
}

bool HTMLPlugInImageElement::isImageType()
{
    if (m_serviceType.isEmpty() && protocolIs(m_url, "data"))
        m_serviceType = mimeTypeFromDataURL(m_url);

    if (Frame* frame = document()->frame()) {
        KURL completedURL = frame->loader()->completeURL(m_url);
        return frame->loader()->client()->objectContentType(completedURL, m_serviceType) == ObjectContentImage;
    }

    return Image::supportsType(m_serviceType);
}

} // namespace WebCore
