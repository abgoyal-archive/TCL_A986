

#ifndef JumpTable_h
#define JumpTable_h

#include "MacroAssembler.h"
#include "UString.h"
#include <wtf/HashMap.h>
#include <wtf/Vector.h>

namespace JSC {

    struct OffsetLocation {
        int32_t branchOffset;
#if ENABLE(JIT)
        CodeLocationLabel ctiOffset;
#endif
    };

    struct StringJumpTable {
        typedef HashMap<RefPtr<UString::Rep>, OffsetLocation> StringOffsetTable;
        StringOffsetTable offsetTable;
#if ENABLE(JIT)
        CodeLocationLabel ctiDefault; // FIXME: it should not be necessary to store this.
#endif

        inline int32_t offsetForValue(UString::Rep* value, int32_t defaultOffset)
        {
            StringOffsetTable::const_iterator end = offsetTable.end();
            StringOffsetTable::const_iterator loc = offsetTable.find(value);
            if (loc == end)
                return defaultOffset;
            return loc->second.branchOffset;
        }

#if ENABLE(JIT)
        inline CodeLocationLabel ctiForValue(UString::Rep* value)
        {
            StringOffsetTable::const_iterator end = offsetTable.end();
            StringOffsetTable::const_iterator loc = offsetTable.find(value);
            if (loc == end)
                return ctiDefault;
            return loc->second.ctiOffset;
        }
#endif
    };

    struct SimpleJumpTable {
        // FIXME: The two Vectors can be combind into one Vector<OffsetLocation>
        Vector<int32_t> branchOffsets;
        int32_t min;
#if ENABLE(JIT)
        Vector<CodeLocationLabel> ctiOffsets;
        CodeLocationLabel ctiDefault;
#endif

        int32_t offsetForValue(int32_t value, int32_t defaultOffset);
        void add(int32_t key, int32_t offset)
        {
            if (!branchOffsets[key])
                branchOffsets[key] = offset;
        }

#if ENABLE(JIT)
        inline CodeLocationLabel ctiForValue(int32_t value)
        {
            if (value >= min && static_cast<uint32_t>(value - min) < ctiOffsets.size())
                return ctiOffsets[value - min];
            return ctiDefault;
        }
#endif
    };

} // namespace JSC

#endif // JumpTable_h
