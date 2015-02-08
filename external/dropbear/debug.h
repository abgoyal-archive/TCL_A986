
#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "includes.h"

/* Debugging */

/* #define DEBUG_VALGRIND */

/*#define DEBUG_TRACE*/


#define CHECKCLEARTOWRITE()

/* (This is Linux specific) */
/*#define DEBUG_FORKGPROF*/

/* A couple of flags, not usually useful, and mightn't do anything */

/*#define DEBUG_KEXHASH*/
/*#define DEBUG_RSA*/

/* you don't need to touch this block */
#ifdef DEBUG_TRACE
#define TRACE(X) dropbear_trace X;
#else /*DEBUG_TRACE*/
#define TRACE(X)
#endif /*DEBUG_TRACE*/

/* #define DEBUG_HACKCRYPT "hL8nrFDt0aJ3E" */ /* this is crypt("password") */

#endif
