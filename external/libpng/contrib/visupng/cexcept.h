


#ifndef CEXCEPT_H
#define CEXCEPT_H


#include <setjmp.h>

#define define_exception_type(etype) \
struct exception_context { \
  jmp_buf *penv; \
  int caught; \
  volatile struct { etype etmp; } v; \
}

/* etmp must be volatile because the application might use automatic */
/* storage for the_exception_context, and etmp is modified between   */
/* the calls to setjmp() and longjmp().  A wrapper struct is used to */
/* avoid warnings about a duplicate volatile qualifier in case etype */
/* already includes it.                                              */

#define init_exception_context(ec) ((void)((ec)->penv = 0))

#define Try \
  { \
    jmp_buf *exception__prev, exception__env; \
    exception__prev = the_exception_context->penv; \
    the_exception_context->penv = &exception__env; \
    if (setjmp(exception__env) == 0) { \
      do

#define exception__catch(action) \
      while (the_exception_context->caught = 0, \
             the_exception_context->caught); \
    } \
    else { \
      the_exception_context->caught = 1; \
    } \
    the_exception_context->penv = exception__prev; \
  } \
  if (!the_exception_context->caught || action) { } \
  else

#define Catch(e) exception__catch(((e) = the_exception_context->v.etmp, 0))
#define Catch_anonymous exception__catch(0)

/* Try ends with do, and Catch begins with while(0) and ends with     */
/* else, to ensure that Try/Catch syntax is similar to if/else        */
/* syntax.                                                            */
/*                                                                    */
/* The 0 in while(0) is expressed as x=0,x in order to appease        */
/* compilers that warn about constant expressions inside while().     */
/* Most compilers should still recognize that the condition is always */
/* false and avoid generating code for it.                            */

#define Throw \
  for (;; longjmp(*the_exception_context->penv, 1)) \
    the_exception_context->v.etmp =


#endif /* CEXCEPT_H */
