

/* dummy dlopen()/dlclose()/dlsym() implementations to be used in static builds */
#include <stddef.h>

void* dlopen(void)
{
    /* Do not return NULL to route around a bug in our SDL configure script */
    /* mimick succesful load, then all calls to dlsym/dlvsym will fail */
    return (void*)"XXX";
}

void dlclose(void)
{
}

void* dlsym(void)
{
    return NULL;
}

void* dlvsym(void)
{
    return NULL;
}

const char* dlerror(void)
{
    return "Dynamic linking not enabled !";
}
