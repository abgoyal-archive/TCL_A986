

#include "config.h"
#include "Font.h"

#include "AffineTransform.h"
#include "GlyphBuffer.h"
#include "Gradient.h"
#include "GraphicsContext.h"
#include "ImageBuffer.h"
#include "Pattern.h"
#include "SimpleFontData.h"

#define SYNTHETIC_OBLIQUE_ANGLE 14

namespace WebCore {

void Font::drawGlyphs(GraphicsContext* context, const SimpleFontData* font, const GlyphBuffer& glyphBuffer,
                      int from, int numGlyphs, const FloatPoint& point) const
{
    cairo_t* cr = context->platformContext();
    cairo_save(cr);

    cairo_set_scaled_font(cr, font->platformData().scaledFont());

    GlyphBufferGlyph* glyphs = (GlyphBufferGlyph*)glyphBuffer.glyphs(from);

    float offset = 0.0f;
    for (int i = 0; i < numGlyphs; i++) {
        glyphs[i].x = offset;
        glyphs[i].y = 0.0f;
        offset += glyphBuffer.advanceAt(from + i);
    }

    Color fillColor = context->fillColor();

    // Synthetic Oblique
    if(font->platformData().syntheticOblique()) {
        cairo_matrix_t mat = {1, 0, -tanf(SYNTHETIC_OBLIQUE_ANGLE * acosf(0) / 90), 1, point.x(), point.y()};
        cairo_transform(cr, &mat);
    } else {
        cairo_translate(cr, point.x(), point.y());
    }

    // Text shadow, inspired by FontMac
    IntSize shadowSize;
    int shadowBlur = 0;
    Color shadowColor;
    bool hasShadow = context->textDrawingMode() == cTextFill &&
        context->getShadow(shadowSize, shadowBlur, shadowColor);

    // TODO: Blur support
    if (hasShadow) {
        // Disable graphics context shadows (not yet implemented) and paint them manually
        context->clearShadow();
        Color shadowFillColor(shadowColor.red(), shadowColor.green(), shadowColor.blue(), shadowColor.alpha() * fillColor.alpha() / 255);
        cairo_save(cr);

        float red, green, blue, alpha;
        shadowFillColor.getRGBA(red, green, blue, alpha);
        cairo_set_source_rgba(cr, red, green, blue, alpha);

#if ENABLE(FILTERS)
        cairo_text_extents_t extents;
        cairo_scaled_font_glyph_extents(font->platformData().scaledFont(), glyphs, numGlyphs, &extents);

        FloatRect rect(FloatPoint(), FloatSize(extents.width, extents.height));
        IntSize shadowBufferSize;
        FloatRect shadowRect;
        float kernelSize = 0.f;
        GraphicsContext::calculateShadowBufferDimensions(shadowBufferSize, shadowRect, kernelSize, rect, shadowSize, shadowBlur);

        // Draw shadow into a new ImageBuffer
        OwnPtr<ImageBuffer> shadowBuffer = ImageBuffer::create(shadowBufferSize);
        GraphicsContext* shadowContext = shadowBuffer->context();
        cairo_t* shadowCr = shadowContext->platformContext();

        cairo_translate(shadowCr, kernelSize, extents.height + kernelSize);

        cairo_set_scaled_font(shadowCr, font->platformData().scaledFont());
        cairo_show_glyphs(shadowCr, glyphs, numGlyphs);
        if (font->syntheticBoldOffset()) {
            cairo_save(shadowCr);
            cairo_translate(shadowCr, font->syntheticBoldOffset(), 0);
            cairo_show_glyphs(shadowCr, glyphs, numGlyphs);
            cairo_restore(shadowCr);
        }
        cairo_translate(cr, 0.0, -extents.height);
        context->createPlatformShadow(shadowBuffer.release(), shadowColor, shadowRect, kernelSize);
#else
        cairo_translate(cr, shadowSize.width(), shadowSize.height());
        cairo_show_glyphs(cr, glyphs, numGlyphs);
        if (font->syntheticBoldOffset()) {
            cairo_save(cr);
            cairo_translate(cr, font->syntheticBoldOffset(), 0);
            cairo_show_glyphs(cr, glyphs, numGlyphs);
            cairo_restore(cr);
        }
#endif

        cairo_restore(cr);
    }

    if (context->textDrawingMode() & cTextFill) {
        if (context->fillGradient()) {
            cairo_set_source(cr, context->fillGradient()->platformGradient());
            if (context->getAlpha() < 1.0f) {
                cairo_push_group(cr);
                cairo_paint_with_alpha(cr, context->getAlpha());
                cairo_pop_group_to_source(cr);
            }
        } else if (context->fillPattern()) {
            AffineTransform affine;
            cairo_set_source(cr, context->fillPattern()->createPlatformPattern(affine));
            if (context->getAlpha() < 1.0f) {
                cairo_push_group(cr);
                cairo_paint_with_alpha(cr, context->getAlpha());
                cairo_pop_group_to_source(cr);
            }
        } else {
            float red, green, blue, alpha;
            fillColor.getRGBA(red, green, blue, alpha);
            cairo_set_source_rgba(cr, red, green, blue, alpha * context->getAlpha());
        }
        cairo_show_glyphs(cr, glyphs, numGlyphs);
        if (font->syntheticBoldOffset()) {
            cairo_save(cr);
            cairo_translate(cr, font->syntheticBoldOffset(), 0);
            cairo_show_glyphs(cr, glyphs, numGlyphs);
            cairo_restore(cr);
        }
    }

    if (context->textDrawingMode() & cTextStroke) {
        if (context->strokeGradient()) {
            cairo_set_source(cr, context->strokeGradient()->platformGradient());
            if (context->getAlpha() < 1.0f) {
                cairo_push_group(cr);
                cairo_paint_with_alpha(cr, context->getAlpha());
                cairo_pop_group_to_source(cr);
            }
        } else if (context->strokePattern()) {
            AffineTransform affine;
            cairo_set_source(cr, context->strokePattern()->createPlatformPattern(affine));
            if (context->getAlpha() < 1.0f) {
                cairo_push_group(cr);
                cairo_paint_with_alpha(cr, context->getAlpha());
                cairo_pop_group_to_source(cr);
            }
        } else {
            Color strokeColor = context->strokeColor();
            float red, green, blue, alpha;
            strokeColor.getRGBA(red, green, blue, alpha);
            cairo_set_source_rgba(cr, red, green, blue, alpha * context->getAlpha());
        } 
        cairo_glyph_path(cr, glyphs, numGlyphs);
        cairo_set_line_width(cr, context->strokeThickness());
        cairo_stroke(cr);
    }

    // Re-enable the platform shadow we disabled earlier
    if (hasShadow)
        context->setShadow(shadowSize, shadowBlur, shadowColor, DeviceColorSpace);

    cairo_restore(cr);
}

}
