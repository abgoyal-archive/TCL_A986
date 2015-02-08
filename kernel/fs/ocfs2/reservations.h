

#ifndef	OCFS2_RESERVATIONS_H
#define	OCFS2_RESERVATIONS_H

#include <linux/rbtree.h>

#define OCFS2_DEFAULT_RESV_LEVEL	2
#define OCFS2_MAX_RESV_LEVEL	9
#define OCFS2_MIN_RESV_LEVEL	0

struct ocfs2_alloc_reservation {
	struct rb_node	r_node;

	unsigned int	r_start;	/* Begining of current window */
	unsigned int	r_len;		/* Length of the window */

	unsigned int	r_last_len;	/* Length of most recent alloc */
	unsigned int	r_last_start;	/* Start of most recent alloc */
	struct list_head	r_lru;	/* LRU list head */

	unsigned int	r_flags;
};

#define	OCFS2_RESV_FLAG_INUSE	0x01	/* Set when r_node is part of a btree */
#define	OCFS2_RESV_FLAG_TMP	0x02	/* Temporary reservation, will be
					 * destroyed immedately after use */
#define	OCFS2_RESV_FLAG_DIR	0x04	/* Reservation is for an unindexed
					 * directory btree */

struct ocfs2_reservation_map {
	struct rb_root		m_reservations;
	char			*m_disk_bitmap;

	struct ocfs2_super	*m_osb;

	/* The following are not initialized to meaningful values until a disk
	 * bitmap is provided. */
	u32			m_bitmap_len;	/* Number of valid
						 * bits available */

	struct list_head	m_lru;		/* LRU of reservations
						 * structures. */

};

void ocfs2_resv_init_once(struct ocfs2_alloc_reservation *resv);

#define OCFS2_RESV_TYPES	(OCFS2_RESV_FLAG_TMP|OCFS2_RESV_FLAG_DIR)
void ocfs2_resv_set_type(struct ocfs2_alloc_reservation *resv,
			 unsigned int flags);

int ocfs2_dir_resv_allowed(struct ocfs2_super *osb);

void ocfs2_resv_discard(struct ocfs2_reservation_map *resmap,
			struct ocfs2_alloc_reservation *resv);


int ocfs2_resmap_init(struct ocfs2_super *osb,
		      struct ocfs2_reservation_map *resmap);

void ocfs2_resmap_restart(struct ocfs2_reservation_map *resmap,
			  unsigned int clen, char *disk_bitmap);

void ocfs2_resmap_uninit(struct ocfs2_reservation_map *resmap);

int ocfs2_resmap_resv_bits(struct ocfs2_reservation_map *resmap,
			   struct ocfs2_alloc_reservation *resv,
			   int *cstart, int *clen);

void ocfs2_resmap_claimed_bits(struct ocfs2_reservation_map *resmap,
			       struct ocfs2_alloc_reservation *resv,
			       u32 cstart, u32 clen);

#endif	/* OCFS2_RESERVATIONS_H */
