

#include "config.h"

#include "ExecutableAllocator.h"

#if ENABLE(ASSEMBLER)

namespace JSC {

size_t ExecutableAllocator::pageSize = 0;

}

#endif // HAVE(ASSEMBLER)
