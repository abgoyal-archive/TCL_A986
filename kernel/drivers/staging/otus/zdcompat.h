
/*                                                                      */
/*  Module Name : zdcompat.h                                            */
/*                                                                      */
/*  Abstract                                                            */
/*     This module contains function definition for compatibility.      */
/*                                                                      */
/*  NOTES                                                               */
/*     Platform dependent.                                              */
/*                                                                      */
/************************************************************************/

#ifndef _ZDCOMPAT_H
#define _ZDCOMPAT_H


#ifndef DECLARE_TASKLET
#define tasklet_schedule(a)   schedule_task(a)
#endif

#undef netdevice_t
typedef struct net_device netdevice_t;

#ifndef in_atomic
#define in_atomic()  0
#endif

#define USB_QUEUE_BULK 0


#endif
