


#ifndef DYNLINK_FUNCTIONS
#error DYNLINK_FUNCTIONS should be defined when including this file
#endif

#ifndef DYNLINK_FUNCTIONS_INIT
#error DYNLINK_FUNCTIONS_INIT should be defined when including this file
#endif

/* just in case */
#undef DYNLINK_FUNC

#define  DYNLINK_FUNC(ret,name,sig) \
      static ret  (*_dynlink_##name) sig ;

#define  DYNLINK_STR(name)   DYNLINK_STR_(name)
#define  DYNLINK_STR_(name)  #name

DYNLINK_FUNCTIONS
#undef DYNLINK_FUNC

static int
DYNLINK_FUNCTIONS_INIT(void*  library)
{
#define  DYNLINK_FUNC(ret,name,sig) \
    do { \
        _dynlink_##name = dlsym( library, DYNLINK_STR(name) ); \
        if (_dynlink_##name == NULL) goto Fail; \
    } while (0);

    DYNLINK_FUNCTIONS
#undef DYNLINK_FUNC

    return 0;
Fail:
    return -1;
}

#ifndef FF
#define FF(name)   (*_dynlink_##name)
#endif

/* clear macros */
#undef DYNLINK_FUNC
#undef DYNLINK_FUNCTIONS
#undef DYNLINK_FUNCTIONS_INIT
