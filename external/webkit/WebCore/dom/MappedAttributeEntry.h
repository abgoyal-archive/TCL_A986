

#ifndef MappedAttributeEntry_h
#define MappedAttributeEntry_h

namespace WebCore {

enum MappedAttributeEntry {
      eNone
    , eUniversal
    , ePersistent
    , eReplaced
    , eBlock
    , eHR
    , eUnorderedList
    , eListItem
    , eTable
    , eCell
    , eCaption
    , eBDO
    , ePre
#if ENABLE(SVG)
    , eSVG
#endif
// When adding new entries, make sure to keep eLastEntry at the end of the list.
    , eLastEntry
};

enum FragmentScriptingPermission { FragmentScriptingAllowed, FragmentScriptingNotAllowed };

}

#endif
