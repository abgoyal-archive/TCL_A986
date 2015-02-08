
#ifndef __NONCONTEXTUALGLYPHSUBSTITUTION_H
#define __NONCONTEXTUALGLYPHSUBSTITUTION_H


#include "LETypes.h"
#include "LayoutTables.h"
#include "LookupTables.h"
#include "MorphTables.h"

U_NAMESPACE_BEGIN

struct NonContextualGlyphSubstitutionHeader : MorphSubtableHeader
{
    LookupTable table;
};

U_NAMESPACE_END
#endif

