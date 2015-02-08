

#ifndef _VIA_VERIFIER_H_
#define _VIA_VERIFIER_H_

typedef enum {
	no_sequence = 0,
	z_address,
	dest_address,
	tex_address
} drm_via_sequence_t;

typedef struct {
	unsigned texture;
	uint32_t z_addr;
	uint32_t d_addr;
	uint32_t t_addr[2][10];
	uint32_t pitch[2][10];
	uint32_t height[2][10];
	uint32_t tex_level_lo[2];
	uint32_t tex_level_hi[2];
	uint32_t tex_palette_size[2];
	uint32_t tex_npot[2];
	drm_via_sequence_t unfinished;
	int agp_texture;
	int multitex;
	struct drm_device *dev;
	drm_local_map_t *map_cache;
	uint32_t vertex_count;
	int agp;
	const uint32_t *buf_start;
} drm_via_state_t;

extern int via_verify_command_stream(const uint32_t * buf, unsigned int size,
				     struct drm_device * dev, int agp);
extern int via_parse_command_stream(struct drm_device *dev, const uint32_t *buf,
				    unsigned int size);

#endif
