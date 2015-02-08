
#include "ucln.h"
#include "ucln_io.h"
#include "umutex.h"
#include "uassert.h"

/* Leave this copyright notice here! It needs to go somewhere in this library. */
static const char copyright[] = U_COPYRIGHT_STRING;

static cleanupFunc *gCleanupFunctions[UCLN_IO_COUNT];

static UBool io_cleanup(void)
{
    ECleanupIOType libType = UCLN_IO_START;

    while (++libType<UCLN_IO_COUNT) {
        if (gCleanupFunctions[libType])
        {
            gCleanupFunctions[libType]();
            gCleanupFunctions[libType] = NULL;
        }
    }
    return TRUE;
}

void ucln_io_registerCleanup(ECleanupIOType type,
                               cleanupFunc *func)
{
    U_ASSERT(UCLN_IO_START < type && type < UCLN_IO_COUNT);
    ucln_registerCleanup(UCLN_IO, io_cleanup);
    if (UCLN_IO_START < type && type < UCLN_IO_COUNT)
    {
        gCleanupFunctions[type] = func;
    }
}

