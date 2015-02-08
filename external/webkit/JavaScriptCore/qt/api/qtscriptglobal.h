

#ifndef qtscriptglobal_h
#define qtscriptglobal_h

#include <QtCore/qglobal.h>

#if defined(Q_OS_WIN) || defined(Q_OS_SYMBIAN)
#  if defined(QT_NODLL)
#  elif defined(QT_MAKEDLL)        /* create a Qt DLL library */
#    if defined(QT_BUILD_JAVASCRIPT_LIB)
#      define Q_JAVASCRIPT_EXPORT Q_DECL_EXPORT
#    else
#      define Q_JAVASCRIPT_EXPORT Q_DECL_IMPORT
#    endif
#  elif defined(QT_DLL) /* use a Qt DLL library */
#    define Q_JAVASCRIPT_EXPORT
#  endif
#endif

#if defined(QT_SHARED)
#  define Q_JAVASCRIPT_EXPORT Q_DECL_EXPORT
#else
#  define Q_JAVASCRIPT_EXPORT
#endif

#endif
