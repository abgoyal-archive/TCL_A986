
#include "LETypes.h"
#include "LayoutTables.h"
#include "LookupTables.h"
#include "LESwaps.h"

U_NAMESPACE_BEGIN

const LookupSegment *BinarySearchLookupTable::lookupSegment(const LookupSegment *segments, LEGlyphID glyph) const
{
    le_int16  unity = SWAPW(unitSize);
    le_int16  probe = SWAPW(searchRange);
    le_int16  extra = SWAPW(rangeShift);
    TTGlyphID ttGlyph = (TTGlyphID) LE_GET_GLYPH(glyph);
    const LookupSegment *entry = segments;
    const LookupSegment *trial = (const LookupSegment *) ((char *) entry + extra);

    if (SWAPW(trial->lastGlyph) <= ttGlyph) {
        entry = trial;
    }

    while (probe > unity) {
        probe >>= 1;
        trial = (const LookupSegment *) ((char *) entry + probe);

        if (SWAPW(trial->lastGlyph) <= ttGlyph) {
            entry = trial;
        }
    }

    if (SWAPW(entry->firstGlyph) <= ttGlyph) {
        return entry;
    }

    return NULL;
}

const LookupSingle *BinarySearchLookupTable::lookupSingle(const LookupSingle *entries, LEGlyphID glyph) const
{
    le_int16  unity = SWAPW(unitSize);
    le_int16  probe = SWAPW(searchRange);
    le_int16  extra = SWAPW(rangeShift);
    TTGlyphID ttGlyph = (TTGlyphID) LE_GET_GLYPH(glyph);
    const LookupSingle *entry = entries;
    const LookupSingle *trial = (const LookupSingle *) ((char *) entry + extra);

    if (SWAPW(trial->glyph) <= ttGlyph) {
        entry = trial;
    }

    while (probe > unity) {
        probe >>= 1;
        trial = (const LookupSingle *) ((char *) entry + probe);

        if (SWAPW(trial->glyph) <= ttGlyph) {
            entry = trial;
        }
    }

    if (SWAPW(entry->glyph) == ttGlyph) {
        return entry;
    }

    return NULL;
}

U_NAMESPACE_END
