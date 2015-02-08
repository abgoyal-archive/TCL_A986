

#ifndef __activity_stats_h
#define __activity_stats_h

#ifdef CONFIG_NET_ACTIVITY_STATS
void activity_stats_update(void);
#else
#define activity_stats_update(void) {}
#endif

#endif /* _NET_ACTIVITY_STATS_H */
