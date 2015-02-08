
#ifndef __RADIOTAP_ITER_H
#define __RADIOTAP_ITER_H

#include "radiotap.h"


struct ieee80211_radiotap_iterator {
	struct ieee80211_radiotap_header *rtheader;
	int max_length;
	int this_arg_index;
	unsigned char *this_arg;

	int arg_index;
	unsigned char *arg;
	uint32_t *next_bitmap;
	uint32_t bitmap_shifter;
};

extern int ieee80211_radiotap_iterator_init(
   struct ieee80211_radiotap_iterator *iterator,
   struct ieee80211_radiotap_header *radiotap_header,
   int max_length);

extern int ieee80211_radiotap_iterator_next(
   struct ieee80211_radiotap_iterator *iterator);

#endif /* __RADIOTAP_ITER_H */
