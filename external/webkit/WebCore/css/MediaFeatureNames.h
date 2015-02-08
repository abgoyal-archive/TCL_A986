
#ifndef MediaFeatureNames_h
#define MediaFeatureNames_h

#include "AtomicString.h"

namespace WebCore {
    namespace MediaFeatureNames {

#define CSS_MEDIAQUERY_NAMES_FOR_EACH_MEDIAFEATURE(macro) \
    macro(color, "color") \
    macro(grid, "grid") \
    macro(monochrome, "monochrome") \
    macro(height, "height") \
    macro(width, "width") \
    macro(orientation, "orientation") \
    macro(aspect_ratio, "aspect-ratio") \
    macro(device_aspect_ratio, "device-aspect-ratio") \
    macro(device_pixel_ratio, "-webkit-device-pixel-ratio") \
    macro(device_height, "device-height") \
    macro(device_width, "device-width") \
    macro(max_color, "max-color") \
    macro(max_aspect_ratio, "max-aspect-ratio") \
    macro(max_device_aspect_ratio, "max-device-aspect-ratio") \
    macro(max_device_pixel_ratio, "-webkit-max-device-pixel-ratio") \
    macro(max_device_height, "max-device-height") \
    macro(max_device_width, "max-device-width") \
    macro(max_height, "max-height") \
    macro(max_monochrome, "max-monochrome") \
    macro(max_width, "max-width") \
    macro(min_color, "min-color") \
    macro(min_aspect_ratio, "min-aspect-ratio") \
    macro(min_device_aspect_ratio, "min-device-aspect-ratio") \
    macro(min_device_pixel_ratio, "-webkit-min-device-pixel-ratio") \
    macro(min_device_height, "min-device-height") \
    macro(min_device_width, "min-device-width") \
    macro(min_height, "min-height") \
    macro(min_monochrome, "min-monochrome") \
    macro(min_width, "min-width") \
    macro(transform_2d, "-webkit-transform-2d") \
    macro(transform_3d, "-webkit-transform-3d") \
    macro(transition, "-webkit-transition") \
    macro(animation, "-webkit-animation") \
// end of macro

#ifndef CSS_MEDIAQUERY_NAMES_HIDE_GLOBALS
    #define CSS_MEDIAQUERY_NAMES_DECLARE(name, str) extern const AtomicString name##MediaFeature;
    CSS_MEDIAQUERY_NAMES_FOR_EACH_MEDIAFEATURE(CSS_MEDIAQUERY_NAMES_DECLARE)
    #undef CSS_MEDIAQUERY_NAMES_DECLARE
#endif

        void init();

    } // namespace MediaFeatureNames
} // namespace WebCore

#endif // MediaFeatureNames_h
