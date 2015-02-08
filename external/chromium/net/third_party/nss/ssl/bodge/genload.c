

#ifdef XP_UNIX
#include <unistd.h>
#define BL_MAXSYMLINKS 20

static char* loader_GetOriginalPathname(const char* link)
{
    char* resolved = NULL;
    char* input = NULL;
    PRUint32 iterations = 0;
    PRInt32 len = 0, retlen = 0;
    if (!link) {
        PR_SetError(PR_INVALID_ARGUMENT_ERROR, 0);
        return NULL;
    }
    len = PR_MAX(1024, strlen(link) + 1);
    resolved = PR_Malloc(len);
    input = PR_Malloc(len);
    if (!resolved || !input) {
        if (resolved) {
            PR_Free(resolved);
        }
        if (input) {
            PR_Free(input);
        }
        return NULL;
    }
    strcpy(input, link);
    while ( (iterations++ < BL_MAXSYMLINKS) &&
            ( (retlen = readlink(input, resolved, len - 1)) > 0) ) {
        char* tmp = input;
        resolved[retlen] = '\0'; /* NULL termination */
        input = resolved;
        resolved = tmp;
    }
    PR_Free(resolved);
    if (iterations == 1 && retlen < 0) {
        PR_Free(input);
        input = NULL;
    }
    return input;
}
#endif /* XP_UNIX */

static PRLibrary *
loader_LoadLibInReferenceDir(const char *referencePath, const char *name)
{
    PRLibrary *dlh = NULL;
    char *fullName = NULL;
    char* c;
    PRLibSpec libSpec;

    /* Remove the trailing filename from referencePath and add the new one */
    c = strrchr(referencePath, PR_GetDirectorySeparator());
    if (c) {
        size_t referencePathSize = 1 + c - referencePath;
        fullName = (char*) PORT_Alloc(strlen(name) + referencePathSize + 1);
        if (fullName) {
            memcpy(fullName, referencePath, referencePathSize);
            strcpy(fullName + referencePathSize, name); 
#ifdef DEBUG_LOADER
            PR_fprintf(PR_STDOUT, "\nAttempting to load fully-qualified %s\n", 
                       fullName);
#endif
            libSpec.type = PR_LibSpec_Pathname;
            libSpec.value.pathname = fullName;
            dlh = PR_LoadLibraryWithFlags(libSpec, PR_LD_NOW | PR_LD_LOCAL);
            PORT_Free(fullName);
        }
    }
    return dlh;
}


static PRLibrary *
loader_LoadLibrary(const char *nameToLoad)
{
    PRLibrary *lib = NULL;
    char* fullPath = NULL;
    PRLibSpec libSpec;

    /* Get the pathname for nameOfAlreadyLoadedLib, i.e. /usr/lib/libnss3.so
     * PR_GetLibraryFilePathname works with either the base library name or a
     * function pointer, depending on the platform. We can't query an exported
     * symbol such as NSC_GetFunctionList, because on some platforms we can't
     * find symbols in loaded implicit dependencies.
     * But we can just get the address of this function !
     */
    fullPath = PR_GetLibraryFilePathname(NameOfThisSharedLib,
                                         (PRFuncPtr)&loader_LoadLibrary);

    if (fullPath) {
        lib = loader_LoadLibInReferenceDir(fullPath, nameToLoad);
#ifdef XP_UNIX
        if (!lib) {
            /*
             * If fullPath is a symbolic link, resolve the symbolic
             * link and try again.
             */
            char* originalfullPath = loader_GetOriginalPathname(fullPath);
            if (originalfullPath) {
                PR_Free(fullPath);
                fullPath = originalfullPath;
                lib = loader_LoadLibInReferenceDir(fullPath, nameToLoad);
            }
        }
#endif
        PR_Free(fullPath);
    }
    if (!lib) {
#ifdef DEBUG_LOADER
        PR_fprintf(PR_STDOUT, "\nAttempting to load %s\n", nameToLoad);
#endif
        libSpec.type = PR_LibSpec_Pathname;
        libSpec.value.pathname = nameToLoad;
        lib = PR_LoadLibraryWithFlags(libSpec, PR_LD_NOW | PR_LD_LOCAL);
    }
    if (NULL == lib) {
#ifdef DEBUG_LOADER
        PR_fprintf(PR_STDOUT, "\nLoading failed : %s.\n", nameToLoad);
#endif
    }
    return lib;
}

