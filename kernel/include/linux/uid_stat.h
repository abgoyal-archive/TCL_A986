

#ifndef __uid_stat_h
#define __uid_stat_h

/* Contains definitions for resource tracking per uid. */

#ifdef CONFIG_UID_STAT
int uid_stat_tcp_snd(uid_t uid, int size);
int uid_stat_tcp_rcv(uid_t uid, int size);
#else
#define uid_stat_tcp_snd(uid, size) do {} while (0);
#define uid_stat_tcp_rcv(uid, size) do {} while (0);
#endif

#endif /* _LINUX_UID_STAT_H */
