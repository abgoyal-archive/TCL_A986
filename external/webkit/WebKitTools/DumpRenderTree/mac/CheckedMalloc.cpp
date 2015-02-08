

#import "config.h" 
#import "CheckedMalloc.h"

#import <malloc/malloc.h>
#import <sys/mman.h>

static void* (*savedMalloc)(malloc_zone_t*, size_t);
static void* (*savedRealloc)(malloc_zone_t*, void*, size_t);

static void* checkedMalloc(malloc_zone_t* zone, size_t size)
{
    if (size >= 0x10000000)
        return 0;
    return savedMalloc(zone, size);
}

static void* checkedRealloc(malloc_zone_t* zone, void* ptr, size_t size)
{
    if (size >= 0x10000000)
        return 0;
    return savedRealloc(zone, ptr, size);
}

void makeLargeMallocFailSilently()
{
    malloc_zone_t* zone = malloc_default_zone();

#if !defined(BUILDING_ON_TIGER) && !defined(BUILDING_ON_LEOPARD) && !defined(BUILDING_ON_SNOW_LEOPARD)
    vm_address_t pageStart = reinterpret_cast<vm_address_t>(zone) & static_cast<vm_size_t>(~(getpagesize() - 1));
    vm_size_t len = reinterpret_cast<vm_address_t>(zone) - pageStart + sizeof(malloc_zone_t);
    mprotect(reinterpret_cast<void*>(pageStart), len, PROT_READ | PROT_WRITE);
#endif

    savedMalloc = zone->malloc;
    savedRealloc = zone->realloc;
    zone->malloc = checkedMalloc;
    zone->realloc = checkedRealloc;

#if !defined(BUILDING_ON_TIGER) && !defined(BUILDING_ON_LEOPARD) && !defined(BUILDING_ON_SNOW_LEOPARD)
    mprotect(reinterpret_cast<void*>(pageStart), len, PROT_READ);
#endif
}
