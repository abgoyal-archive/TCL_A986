


#include "debuglib.h"

#ifdef DIVA_NO_DEBUGLIB
static DIVA_DI_PRINTF dprintf;
#else /* DIVA_NO_DEBUGLIB */
 
_DbgHandle_ myDriverDebugHandle = { 0 /*!Registered*/, DBG_HANDLE_VERSION };
DIVA_DI_PRINTF dprintf = no_printf;
/*****************************************************************************/
#define DBG_FUNC(name) \
void  \
myDbgPrint_##name (char *format, ...) \
{ va_list ap ; \
 if ( myDriverDebugHandle.dbg_prt ) \
 { va_start (ap, format) ; \
  (myDriverDebugHandle.dbg_prt) \
   (myDriverDebugHandle.id, DLI_##name, format, ap) ; \
  va_end (ap) ; \
} }
DBG_FUNC(LOG)
DBG_FUNC(FTL)
DBG_FUNC(ERR)
DBG_FUNC(TRC)
DBG_FUNC(MXLOG)
DBG_FUNC(FTL_MXLOG)
void 
myDbgPrint_EVL (long msgID, ...)
{ va_list ap ;
 if ( myDriverDebugHandle.dbg_ev )
 { va_start (ap, msgID) ;
  (myDriverDebugHandle.dbg_ev)
   (myDriverDebugHandle.id, (unsigned long)msgID, ap) ;
  va_end (ap) ;
} }
DBG_FUNC(REG)
DBG_FUNC(MEM)
DBG_FUNC(SPL)
DBG_FUNC(IRP)
DBG_FUNC(TIM)
DBG_FUNC(BLK)
DBG_FUNC(TAPI)
DBG_FUNC(NDIS)
DBG_FUNC(CONN)
DBG_FUNC(STAT)
DBG_FUNC(SEND)
DBG_FUNC(RECV)
DBG_FUNC(PRV0)
DBG_FUNC(PRV1)
DBG_FUNC(PRV2)
DBG_FUNC(PRV3)
/*****************************************************************************/
int
DbgRegister (char *drvName, char *drvTag, unsigned long dbgMask)
{
 int len;
 DbgDeregister () ;
 myDriverDebugHandle.Version = DBG_HANDLE_VERSION ;
 myDriverDebugHandle.id  = -1 ;
 myDriverDebugHandle.dbgMask = dbgMask | (DL_EVL | DL_FTL | DL_LOG) ;
 len = strlen (drvName) ;
 memcpy (myDriverDebugHandle.drvName, drvName,
         (len < sizeof(myDriverDebugHandle.drvName)) ?
    len : sizeof(myDriverDebugHandle.drvName) - 1) ;
 len = strlen (drvTag) ;
 memcpy (myDriverDebugHandle.drvTag, drvTag,
         (len < sizeof(myDriverDebugHandle.drvTag)) ?
    len : sizeof(myDriverDebugHandle.drvTag) - 1) ;
 dprintf("\000\377", &myDriverDebugHandle) ;
 if ( myDriverDebugHandle.dbg_prt )
 {
  return (1) ;
 }
 if ( myDriverDebugHandle.dbg_end != NULL
   /* location of 'dbg_prt' in _OldDbgHandle_ struct */
   && (myDriverDebugHandle.regTime.LowPart ||
       myDriverDebugHandle.regTime.HighPart  ) )
   /* same location as in _OldDbgHandle_ struct */
 {
  dprintf("%s: Cannot log to old maint driver !", drvName) ;
  myDriverDebugHandle.dbg_end =
  ((_OldDbgHandle_ *)&myDriverDebugHandle)->dbg_end ;
  DbgDeregister () ;
 }
 return (0) ;
}
/*****************************************************************************/
void
DbgSetLevel (unsigned long dbgMask)
{
 myDriverDebugHandle.dbgMask = dbgMask | (DL_EVL | DL_FTL | DL_LOG) ;
}
/*****************************************************************************/
void
DbgDeregister (void)
{
 if ( myDriverDebugHandle.dbg_end )
 {
  (myDriverDebugHandle.dbg_end)(&myDriverDebugHandle) ;
 }
 memset (&myDriverDebugHandle, 0, sizeof(myDriverDebugHandle)) ;
}
void  xdi_dbg_xlog (char* x, ...) {
 va_list ap;
 va_start (ap, x);
 if (myDriverDebugHandle.dbg_end &&
   (myDriverDebugHandle.dbg_irq || myDriverDebugHandle.dbg_old) &&
   (myDriverDebugHandle.dbgMask & DL_STAT)) {
  if (myDriverDebugHandle.dbg_irq) {
   (*(myDriverDebugHandle.dbg_irq))(myDriverDebugHandle.id,
       (x[0] != 0) ? DLI_TRC : DLI_XLOG, x, ap);
  } else {
   (*(myDriverDebugHandle.dbg_old))(myDriverDebugHandle.id, x, ap);
  }
 }
 va_end(ap);
}
/*****************************************************************************/
#endif /* DIVA_NO_DEBUGLIB */
