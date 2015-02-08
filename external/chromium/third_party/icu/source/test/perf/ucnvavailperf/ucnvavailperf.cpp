
#include <malloc.h>
#include <stdio.h>
#include "unicode/utypes.h"
#include "unicode/putil.h"
#include "unicode/uclean.h"
#include "unicode/ucnv.h"
#include "unicode/utimer.h"

static size_t icuMemUsage = 0;

U_CDECL_BEGIN

void *U_CALLCONV
my_alloc(const void *context, size_t size) {
    size_t *p = (size_t *)malloc(size + sizeof(size_t));
    if (p != NULL) {
        icuMemUsage += size;
        *p = size;
        return p + 1;
    } else {
        return NULL;
    }
}

void U_CALLCONV
my_free(const void *context, void *mem) {
    if (mem != NULL) {
        const size_t *p = (const size_t *)mem - 1;
        icuMemUsage -= *p;
        free((void *)p);
    }
}

// Not used in the common library.
void *U_CALLCONV
my_realloc(const void *context, void *mem, size_t size) {
    my_free(context, mem);
    return NULL;
}

U_CDECL_END

int main(int argc, const char *argv[]) {
    UErrorCode errorCode = U_ZERO_ERROR;

    // Hook in our own memory allocation functions so that we can measure
    // the memory usage.
    u_setMemoryFunctions(NULL, my_alloc, my_realloc, my_free, &errorCode);
    if(U_FAILURE(errorCode)) {
        fprintf(stderr,
                "u_setMemoryFunctions() failed - %s\n",
                u_errorName(errorCode));
        return errorCode;
    }

    if (argc > 1) {
        printf("u_setDataDirectory(%s)\n", argv[1]);
        u_setDataDirectory(argv[1]);
    }

    // Preload a purely algorithmic converter via an alias,
    // to make sure that relevant data can be loaded and to set up
    // caches and such that are needed even if none of the data-driven
    // converters needs to be loaded.
    ucnv_close(ucnv_open("ibm-1208", &errorCode));
    if(U_FAILURE(errorCode)) {
        fprintf(stderr,
                "unable to open UTF-8 converter via an alias - %s\n",
                u_errorName(errorCode));
        return errorCode;
    }

    printf("memory usage after ucnv_open(ibm-1208): %lu\n", (long)icuMemUsage);

    UTimer start_time;
    utimer_getTime(&start_time);
    // Measure the time to find out the list of actually available converters.
    int32_t count = ucnv_countAvailable();
    double elapsed = utimer_getElapsedSeconds(&start_time);
    printf("ucnv_countAvailable() reports that %d converters are available.\n", count);
    printf("ucnv_countAvailable() took %g seconds to figure this out.\n", elapsed);
    printf("memory usage after ucnv_countAvailable(): %lu\n", (long)icuMemUsage);

    ucnv_flushCache();
    printf("memory usage after ucnv_flushCache(): %lu\n", (long)icuMemUsage);

    u_cleanup();
    printf("memory usage after u_cleanup(): %lu\n", (long)icuMemUsage);

    return 0;
}
