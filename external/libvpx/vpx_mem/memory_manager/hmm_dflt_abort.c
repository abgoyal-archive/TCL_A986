



#include "hmm_intrnl.h"
#include <stdio.h>
#include <stdlib.h>

static int entered = 0;

void hmm_dflt_abort(const char *file, const char *line)
{
    /* Avoid use of printf(), which is more likely to use heap. */

    if (entered)

        /* The standard I/O functions called a heap function and caused
        ** an indirect recursive call to this function.  So we'll have
        ** to just exit without printing a message.  */
        while (1);

    entered = 1;

    fputs("\n_abort - Heap corruption\n" "File: ", stderr);
    fputs(file, stderr);
    fputs("  Line: ", stderr);
    fputs(line, stderr);
    fputs("\n\n", stderr);
    fputs("hmm_dflt_abort: while(1)!!!\n", stderr);
    fflush(stderr);

    while (1);
}
