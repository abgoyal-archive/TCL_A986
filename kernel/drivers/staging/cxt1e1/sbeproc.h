

#ifndef _INC_SBEPROC_H_
#define _INC_SBEPROC_H_



#ifdef CONFIG_PROC_FS
#ifdef __KERNEL__
void        sbecom_proc_brd_cleanup (ci_t *);
int __init  sbecom_proc_brd_init (ci_t *);

#endif                          /*** __KERNEL__ ***/
#endif                          /*** CONFIG_PROC_FS ***/
#endif                          /*** _INC_SBEPROC_H_ ***/
