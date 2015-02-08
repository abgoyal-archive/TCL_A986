

#include "config.h"

#ifndef BUILDING_ON_TIGER

#include "PurgeableBuffer.h"

#include <mach/mach.h>
#include <wtf/Assertions.h>

namespace WebCore {
    
static const size_t minPurgeableBufferSize = 4096; // one page
    
PurgeableBuffer::PurgeableBuffer(char* data, size_t size)
    : m_data(data)
    , m_size(size)
    , m_purgePriority(PurgeDefault)
    , m_state(NonVolatile)
{
}
    
PurgeableBuffer::~PurgeableBuffer()
{
    vm_deallocate(mach_task_self(), reinterpret_cast<vm_address_t>(m_data), m_size);
}

PurgeableBuffer* PurgeableBuffer::create(const char* data, size_t size)
{
    if (size < minPurgeableBufferSize)
        return 0;
    
    vm_address_t buffer = 0;
    kern_return_t ret = vm_allocate(mach_task_self(), &buffer, size, VM_FLAGS_PURGABLE | VM_FLAGS_ANYWHERE);

    ASSERT(ret == KERN_SUCCESS);
    if (ret != KERN_SUCCESS)
        return 0;

    ret = vm_copy(mach_task_self(), reinterpret_cast<vm_address_t>(data), size, buffer);

    ASSERT(ret == KERN_SUCCESS);
    if (ret != KERN_SUCCESS) {
        vm_deallocate(mach_task_self(), buffer, size);
        return 0;
    }

    return new PurgeableBuffer(reinterpret_cast<char*>(buffer), size);
}

bool PurgeableBuffer::makePurgeable(bool purgeable)
{
    if (purgeable) {
        if (m_state != NonVolatile)
            return true;

        int volatileGroup;
        if (m_purgePriority == PurgeFirst)
            volatileGroup = VM_VOLATILE_GROUP_0;
        else if (m_purgePriority == PurgeMiddle)
            volatileGroup = VM_VOLATILE_GROUP_4;
        else
            volatileGroup = VM_VOLATILE_GROUP_7;
        
        int state = VM_PURGABLE_VOLATILE | volatileGroup;
        // So apparently "purgeable" is the correct spelling and the API here is misspelled.
        kern_return_t ret = vm_purgable_control(mach_task_self(), reinterpret_cast<vm_address_t>(m_data), VM_PURGABLE_SET_STATE, &state);
        
        if (ret != KERN_SUCCESS) {
            // If that failed we have no clue what state we are in so assume purged.
            m_state = Purged;
            return true;
        }
        
        m_state = Volatile;
        return true;
    }

    if (m_state == NonVolatile)
        return true;
    if (m_state == Purged)
        return false;
    
    int state = VM_PURGABLE_NONVOLATILE;
    kern_return_t ret = vm_purgable_control(mach_task_self(), reinterpret_cast<vm_address_t>(m_data), VM_PURGABLE_SET_STATE, &state);

    if (ret != KERN_SUCCESS) {
        // If that failed we have no clue what state we are in so assume purged.
        m_state = Purged;
        return false;
    }

    m_state = state & VM_PURGABLE_EMPTY ? Purged : NonVolatile;
    return m_state == NonVolatile;
}
    
bool PurgeableBuffer::wasPurged() const
{
    if (m_state == NonVolatile)
        return false;
    if (m_state == Purged)
        return true;

    int state;
    kern_return_t ret = vm_purgable_control(mach_task_self(), reinterpret_cast<vm_address_t>(m_data), VM_PURGABLE_GET_STATE, &state);

    if (ret != KERN_SUCCESS) {
        // If that failed we have no clue what state we are in so assume purged.
        m_state = Purged;
        return true;        
    }

    if (state & VM_PURGABLE_EMPTY) {
        m_state = Purged;
        return true;
    }
        
    return false;
}

void PurgeableBuffer::setPurgePriority(PurgePriority priority)
{
    if (priority == m_purgePriority)
        return;
    m_purgePriority = priority;
    if (m_state != Volatile)
        return;
    m_state = NonVolatile;
    makePurgeable(true);
}
    
const char* PurgeableBuffer::data() const
{
    ASSERT(m_state == NonVolatile);
    return m_data;
}
    
}

#endif // BUILDING_ON_TIGER
