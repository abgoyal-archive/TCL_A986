

#ifndef _UPD64083_H_
#define _UPD64083_H_


/* Operating modes: */

/* YCS mode: Y/C separation (burst locked clocking) */
#define UPD64083_YCS_MODE      0
/* YCS+ mode: 2D Y/C separation and YCNR (burst locked clocking) */
#define UPD64083_YCS_PLUS_MODE 1

/* MNNR mode: frame comb type YNR+C delay (line locked clocking) */
#define UPD64083_MNNR_MODE     2
/* YCNR mode: frame recursive YCNR (burst locked clocking) */
#define UPD64083_YCNR_MODE     3

#define UPD64083_EXT_Y_ADC     (1 << 2)

#endif
