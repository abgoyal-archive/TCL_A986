

#include "config.h"
#include "JumpTable.h"

namespace JSC {

int32_t SimpleJumpTable::offsetForValue(int32_t value, int32_t defaultOffset)
{
    if (value >= min && static_cast<uint32_t>(value - min) < branchOffsets.size()) {
        int32_t offset = branchOffsets[value - min];
        if (offset)
            return offset;
    }
    return defaultOffset;        
}

} // namespace JSC
