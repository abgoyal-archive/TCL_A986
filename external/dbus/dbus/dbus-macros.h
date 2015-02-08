
/* -*- mode: C; c-file-style: "gnu" -*- */
#if !defined (DBUS_INSIDE_DBUS_H) && !defined (DBUS_COMPILATION)
#error "Only <dbus/dbus.h> can be included directly, this file may disappear or change contents."
#endif

#ifndef DBUS_MACROS_H
#define DBUS_MACROS_H

#ifdef  __cplusplus
#  define DBUS_BEGIN_DECLS  extern "C" {
#  define DBUS_END_DECLS    }
#else
#  define DBUS_BEGIN_DECLS
#  define DBUS_END_DECLS
#endif

#ifndef TRUE
#  define TRUE 1
#endif
#ifndef FALSE
#  define FALSE 0
#endif

#ifndef NULL
#  ifdef __cplusplus
#    define NULL        (0L)
#  else /* !__cplusplus */
#    define NULL        ((void*) 0)
#  endif /* !__cplusplus */
#endif

#if  __GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 1)
#define DBUS_GNUC_DEPRECATED __attribute__((__deprecated__))
#else
#define DBUS_GNUC_DEPRECATED
#endif

/* Normally docs are in .c files, but there isn't a .c file for this. */


/** @} */

#endif /* DBUS_MACROS_H */
