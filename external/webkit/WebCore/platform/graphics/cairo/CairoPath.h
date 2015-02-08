

#ifndef CairoPath_h
#define CairoPath_h

#include <cairo.h>

namespace WebCore {

    // This is necessary since cairo_path_fixed_t isn't exposed in Cairo's public API.
    struct CairoPath {
        cairo_t* m_cr;

        CairoPath()
        {
            static cairo_surface_t* pathSurface = cairo_image_surface_create(CAIRO_FORMAT_A8, 1, 1);
            m_cr = cairo_create(pathSurface);
        }

        ~CairoPath()
        {
            cairo_destroy(m_cr);
        }
    };

} // namespace WebCore

#endif // CairoPath_h
