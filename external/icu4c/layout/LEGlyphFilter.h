
#ifndef __LEGLYPHFILTER__H
#define __LEGLYPHFILTER__H

#include "LETypes.h"

U_NAMESPACE_BEGIN

class LEGlyphFilter /* not : public UObject because this is an interface/mixin class */ {
public:
    /**
     * Destructor.
     * @internal
     */
    virtual ~LEGlyphFilter();

    /**
     * This method is used to test a particular
     * glyph index to see if it is in the set
     * recognized by the filter.
     *
     * @param glyph - the glyph index to be tested
     *
     * @return TRUE if the glyph index is in the set.
     *
     * @internal
     */
    virtual le_bool accept(LEGlyphID glyph) const = 0;
};

U_NAMESPACE_END
#endif
