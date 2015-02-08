

#include "config.h"
#include "DragImage.h"

#include "CachedImage.h"
#include "Image.h"

#include <gtk/gtk.h>

namespace WebCore {

IntSize dragImageSize(DragImageRef image)
{
    if (image)
        return IntSize(gdk_pixbuf_get_width(image), gdk_pixbuf_get_height(image));

    return IntSize(0, 0);
}

void deleteDragImage(DragImageRef image)
{
    if (image)
        g_object_unref(image);
}

DragImageRef scaleDragImage(DragImageRef image, FloatSize scale)
{
    if (image) {
        IntSize imageSize = dragImageSize(image);
        GdkPixbuf* scaledImage = gdk_pixbuf_scale_simple(image,
                                                         imageSize.width() * scale.width(),
                                                         imageSize.height() * scale.height(),
                                                         GDK_INTERP_BILINEAR);
        deleteDragImage(image);
        return scaledImage;
    }

    return 0;
}

DragImageRef dissolveDragImageToFraction(DragImageRef image, float)
{
    return image;
}

DragImageRef createDragImageFromImage(Image* image)
{
    return image->getGdkPixbuf();
}

DragImageRef createDragImageIconForCachedImage(CachedImage*)
{
    return 0;
}

}
