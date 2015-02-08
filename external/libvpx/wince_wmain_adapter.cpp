


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_NUM_ARG 64
#define MAX_SIZ_ARG 512

extern "C"
{
    int main(int argc, char **argv);
}

int wmain(int argc, wchar_t **argv) {
    char *cargv[MAX_NUM_ARG];
    char chargv[MAX_SIZ_ARG];
    int ret;

    /* transform command line arguments from (wchar_t *) to (char *) */
    for(int i=0; i<argc; i++) {
        wcstombs( chargv, argv[i], sizeof(chargv));
        cargv[i] = _strdup(chargv);
    }

    ret = main(argc, (char **)cargv);

    //free the memory located by _strdup()
    for(int i=0; i<argc; i++)
        free(cargv[i]);

    return ret;
}
