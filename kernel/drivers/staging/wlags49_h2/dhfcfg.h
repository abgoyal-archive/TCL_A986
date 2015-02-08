

#ifndef DHFCFG_H
#define DHFCFG_H


#define DHF_WCI
/* !!!#define DHF_UIL */

#ifdef USE_BIG_ENDIAN
#define DHF_BIG_ENDIAN
#else
#define DHF_LITTLE_ENDIAN
#endif  /* USE_BIG_ENDIAN */



#define _INC_STDLIB
#define _INC_STRING







#define DSF_VOLATILE_ONLY

/* Define DSF_HERMESII if you want to use the DHF for the Hermes-II */
#ifdef HERMES2
#define DSF_HERMESII
#else
#undef DSF_HERMESII
#endif /* HERMES2 */


#endif /* DHFCFG_H */
