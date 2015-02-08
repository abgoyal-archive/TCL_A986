
#ifndef __CURSIVEATTACHMENTSUBTABLES_H
#define __CURSIVEATTACHMENTSUBTABLES_H


#include "LETypes.h"
#include "OpenTypeTables.h"
#include "GlyphPositioningTables.h"

U_NAMESPACE_BEGIN

class LEFontInstance;
class GlyphIterator;

struct EntryExitRecord
{
    Offset entryAnchor;
    Offset exitAnchor;
};

struct CursiveAttachmentSubtable : GlyphPositioningSubtable
{
    le_uint16 entryExitCount;
    EntryExitRecord entryExitRecords[ANY_NUMBER];

    le_uint32  process(GlyphIterator *glyphIterator, const LEFontInstance *fontInstance) const;
};

U_NAMESPACE_END
#endif


