
#ifndef _MDP_SCALE_TABLES_H_
#define _MDP_SCALE_TABLES_H_

#include <linux/types.h>
struct mdp_table_entry {
	uint32_t reg;
	uint32_t val;
};

extern struct mdp_table_entry mdp_upscale_table[64];

enum {
	MDP_DOWNSCALE_PT2TOPT4,
	MDP_DOWNSCALE_PT4TOPT6,
	MDP_DOWNSCALE_PT6TOPT8,
	MDP_DOWNSCALE_PT8TO1,
	MDP_DOWNSCALE_MAX,
};

extern struct mdp_table_entry *mdp_downscale_x_table[MDP_DOWNSCALE_MAX];
extern struct mdp_table_entry *mdp_downscale_y_table[MDP_DOWNSCALE_MAX];
extern struct mdp_table_entry mdp_gaussian_blur_table[];

#endif
