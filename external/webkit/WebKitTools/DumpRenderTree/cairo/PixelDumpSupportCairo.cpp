

#include "config.h"
#include "PixelDumpSupportCairo.h"

#include "DumpRenderTree.h"
#include "PixelDumpSupport.h"
#include <algorithm>
#include <ctype.h>
#include <wtf/Assertions.h>
#include <wtf/RefPtr.h>
#include <wtf/RetainPtr.h>
#include <wtf/StringExtras.h>

#if PLATFORM(WIN)
#include "MD5.h"
#endif

using namespace std;

static cairo_status_t writeFunction(void* closure, const unsigned char* data, unsigned int length)
{
    Vector<unsigned char>* in = reinterpret_cast<Vector<unsigned char>*>(closure);
    in->append(data, length);
    return CAIRO_STATUS_SUCCESS;
}

static void printPNG(cairo_surface_t* image)
{
    Vector<unsigned char> pixelData;
    // Only PNG output is supported for now.
    cairo_surface_write_to_png_stream(image, writeFunction, &pixelData);

    const size_t dataLength = pixelData.size();
    const unsigned char* data = pixelData.data();

    printPNG(data, dataLength);
}

void computeMD5HashStringForBitmapContext(BitmapContext* context, char hashString[33])
{
    cairo_t* bitmapContext = context->cairoContext();
    cairo_surface_t* surface = cairo_get_target(bitmapContext);

    ASSERT(cairo_image_surface_get_format(surface) == CAIRO_FORMAT_ARGB32); // ImageDiff assumes 32 bit RGBA, we must as well.

    size_t pixelsHigh = cairo_image_surface_get_height(surface);
    size_t pixelsWide = cairo_image_surface_get_width(surface);
    size_t bytesPerRow = pixelsWide * cairo_image_surface_get_stride(surface);

    MD5_CTX md5Context;
    MD5_Init(&md5Context);
    unsigned char* bitmapData = static_cast<unsigned char*>(cairo_image_surface_get_data(surface));
    for (unsigned row = 0; row < pixelsHigh; row++) {
        MD5_Update(&md5Context, bitmapData, 4 * pixelsWide);
        bitmapData += bytesPerRow;
    }
    unsigned char hash[16];
    MD5_Final(hash, &md5Context);

    hashString[0] = '\0';
    for (int i = 0; i < 16; i++)
        snprintf(hashString, 33, "%s%02x", hashString, hash[i]);
}

void dumpBitmap(BitmapContext* context)
{
    cairo_surface_t* surface = cairo_get_target(context->cairoContext());
    printPNG(surface);
}
