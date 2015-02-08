
#ifndef Expat_External_INCLUDED
#define Expat_External_INCLUDED 1

/* External API definitions */

#if defined(_MSC_EXTENSIONS) && !defined(__BEOS__) && !defined(__CYGWIN__)
#define XML_USE_MSC_EXTENSIONS 1
#endif

#ifndef XMLCALL
#if defined(_MSC_VER)
#define XMLCALL __cdecl
#elif defined(__GNUC__) && defined(__i386) && !defined(__INTEL_COMPILER)
#define XMLCALL __attribute__((cdecl))
#else
#define XMLCALL
#endif
#endif  /* not defined XMLCALL */


#if !defined(XML_STATIC) && !defined(XMLIMPORT)
#ifndef XML_BUILDING_EXPAT
/* using Expat from an application */

#ifdef XML_USE_MSC_EXTENSIONS
#define XMLIMPORT __declspec(dllimport)
#endif

#endif
#endif  /* not defined XML_STATIC */


/* If we didn't define it above, define it away: */
#ifndef XMLIMPORT
#define XMLIMPORT
#endif


#define XMLPARSEAPI(type) XMLIMPORT type XMLCALL

#ifdef __cplusplus
extern "C" {
#endif

#ifdef XML_UNICODE_WCHAR_T
#define XML_UNICODE
#endif

#ifdef XML_UNICODE     /* Information is UTF-16 encoded. */
#ifdef XML_UNICODE_WCHAR_T
typedef wchar_t XML_Char;
typedef wchar_t XML_LChar;
#else
typedef unsigned short XML_Char;
typedef char XML_LChar;
#endif /* XML_UNICODE_WCHAR_T */
#else                  /* Information is UTF-8 encoded. */
typedef char XML_Char;
typedef char XML_LChar;
#endif /* XML_UNICODE */

#ifdef XML_LARGE_SIZE  /* Use large integers for file/stream positions. */
#if defined(XML_USE_MSC_EXTENSIONS) && _MSC_VER < 1400
typedef __int64 XML_Index; 
typedef unsigned __int64 XML_Size;
#else
typedef long long XML_Index;
typedef unsigned long long XML_Size;
#endif
#else
typedef long XML_Index;
typedef unsigned long XML_Size;
#endif /* XML_LARGE_SIZE */

#ifdef __cplusplus
}
#endif

#endif /* not Expat_External_INCLUDED */
