

#ifndef _DEBUG_H
#define _DEBUG_H




/* Turn on debugging here if not done with a preprocessor define */
#ifndef DBG
#define DBG 0
#else
#undef	DBG
#define DBG 1
#endif /* DBG */




#if DBG
/****************************************************************************/

#ifndef DBG_LVL
#define DBG_LVL 5			/* yields nothing via init_module,
							   original value of 5 yields DBG_TRACE_ON and DBG_VERBOSE_ON */
#endif  /*  DBG_LVL*/


#define DBG_ERROR_ON        0x00000001L
#define DBG_WARNING_ON      0x00000002L
#define DBG_NOTICE_ON       0x00000004L
#define DBG_TRACE_ON        0x00000008L
#define DBG_VERBOSE_ON      0x00000010L
#define DBG_PARAM_ON        0x00000020L
#define DBG_BREAK_ON        0x00000040L
#define DBG_RX_ON           0x00000100L
#define DBG_TX_ON           0x00000200L
#define DBG_DS_ON           0x00000400L

#define DBG_DEFAULTS        (DBG_ERROR_ON | DBG_WARNING_ON | DBG_BREAK_ON)

#define DBG_FLAGS(A)        ((A)->DebugFlag)
#define DBG_NAME(A)         ((A)->dbgName)
#define DBG_LEVEL(A)        ((A)->dbgLevel)


#ifndef PRINTK
#   define PRINTK(S...)     printk(S)
#endif /* PRINTK */


#ifndef DBG_PRINT
#   define DBG_PRINT(S...)  PRINTK(KERN_DEBUG S)
#endif /* DBG_PRINT */


#ifndef DBG_PRINTC
#   define DBG_PRINTC(S...) PRINTK(S)
#endif /* DBG_PRINTC */


#ifndef DBG_TRAP
#   define DBG_TRAP         {}
#endif /* DBG_TRAP */


#define _ENTER_STR          ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
#define _LEAVE_STR          "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"


#define _DBG_ENTER(A)       DBG_PRINT("%s:%.*s:%s\n", DBG_NAME(A), ++DBG_LEVEL(A), _ENTER_STR, __FUNC__)
#define _DBG_LEAVE(A)       DBG_PRINT("%s:%.*s:%s\n", DBG_NAME(A), DBG_LEVEL(A)--, _LEAVE_STR, __FUNC__)


#define DBG_FUNC(F)         static const char *__FUNC__ = F;

#define DBG_ENTER(A)        {if (DBG_FLAGS(A) & DBG_TRACE_ON) \
				_DBG_ENTER(A); }

#define DBG_LEAVE(A)        {if (DBG_FLAGS(A) & DBG_TRACE_ON) \
				 _DBG_LEAVE(A); }

#define DBG_PARAM(A, N, F, S...)   {if (DBG_FLAGS(A) & DBG_PARAM_ON) \
				DBG_PRINT("  %s -- "F"\n", N, S); }


#define DBG_ERROR(A, S...)   {if (DBG_FLAGS(A) & DBG_ERROR_ON) {\
				DBG_PRINT("%s:ERROR:%s ", DBG_NAME(A), __FUNC__);\
				DBG_PRINTC(S); \
				DBG_TRAP; \
				} \
				}


#define DBG_WARNING(A, S...) {if (DBG_FLAGS(A) & DBG_WARNING_ON) {\
				DBG_PRINT("%s:WARNING:%s ", DBG_NAME(A), __FUNC__);\
				DBG_PRINTC(S); } }


#define DBG_NOTICE(A, S...)  {if (DBG_FLAGS(A) & DBG_NOTICE_ON) {\
				DBG_PRINT("%s:NOTICE:%s ", DBG_NAME(A), __FUNC__);\
				DBG_PRINTC(S); \
				} \
				}


#define DBG_TRACE(A, S...)   do {if (DBG_FLAGS(A) & DBG_TRACE_ON) {\
				DBG_PRINT("%s:%s ", DBG_NAME(A), __FUNC__);\
				DBG_PRINTC(S); } } while (0)


#define DBG_RX(A, S...)      {if (DBG_FLAGS(A) & DBG_RX_ON) {\
				DBG_PRINT(S); } }


#define DBG_TX(A, S...)      {if (DBG_FLAGS(A) & DBG_TX_ON) {\
				DBG_PRINT(S); } }

#define DBG_DS(A, S...)      {if (DBG_FLAGS(A) & DBG_DS_ON) {\
				DBG_PRINT(S); } }


#define DBG_ASSERT(C)		{ \
				if (!(C)) {\
					DBG_PRINT("ASSERT(%s) -- %s#%d (%s)\n", \
					#C, __FILE__, __LINE__, __FUNC__); \
					DBG_TRAP; \
					} \
					}

typedef struct {
    char           *dbgName;
    int             dbgLevel;
    unsigned long   DebugFlag;
} dbg_info_t;


/****************************************************************************/
#else /* DBG */
/****************************************************************************/

#define DBG_DEFN
#define DBG_TRAP
#define DBG_FUNC(F)
#define DBG_PRINT(S...)
#define DBG_ENTER(A)
#define DBG_LEAVE(A)
#define DBG_PARAM(A, N, F, S...)
#define DBG_ERROR(A, S...)
#define DBG_WARNING(A, S...)
#define DBG_NOTICE(A, S...)
#define DBG_TRACE(A, S...)
#define DBG_RX(A, S...)
#define DBG_TX(A, S...)
#define DBG_DS(A, S...)
#define DBG_ASSERT(C)

#endif /* DBG */
/****************************************************************************/




#endif /* _DEBUG_H */

