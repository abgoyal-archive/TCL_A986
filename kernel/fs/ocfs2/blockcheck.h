

#ifndef OCFS2_BLOCKCHECK_H
#define OCFS2_BLOCKCHECK_H


/* Count errors and error correction from blockcheck.c */
struct ocfs2_blockcheck_stats {
	spinlock_t b_lock;
	u64 b_check_count;	/* Number of blocks we've checked */
	u64 b_failure_count;	/* Number of failed checksums */
	u64 b_recover_count;	/* Number of blocks fixed by ecc */

	/*
	 * debugfs entries, used if this is passed to
	 * ocfs2_blockcheck_stats_debugfs_install()
	 */
	struct dentry *b_debug_dir;	/* Parent of the debugfs  files */
	struct dentry *b_debug_check;	/* Exposes b_check_count */
	struct dentry *b_debug_failure;	/* Exposes b_failure_count */
	struct dentry *b_debug_recover;	/* Exposes b_recover_count */
};


/* High level block API */
void ocfs2_compute_meta_ecc(struct super_block *sb, void *data,
			    struct ocfs2_block_check *bc);
int ocfs2_validate_meta_ecc(struct super_block *sb, void *data,
			    struct ocfs2_block_check *bc);
void ocfs2_compute_meta_ecc_bhs(struct super_block *sb,
				struct buffer_head **bhs, int nr,
				struct ocfs2_block_check *bc);
int ocfs2_validate_meta_ecc_bhs(struct super_block *sb,
				struct buffer_head **bhs, int nr,
				struct ocfs2_block_check *bc);

/* Lower level API */
void ocfs2_block_check_compute(void *data, size_t blocksize,
			       struct ocfs2_block_check *bc);
int ocfs2_block_check_validate(void *data, size_t blocksize,
			       struct ocfs2_block_check *bc,
			       struct ocfs2_blockcheck_stats *stats);
void ocfs2_block_check_compute_bhs(struct buffer_head **bhs, int nr,
				   struct ocfs2_block_check *bc);
int ocfs2_block_check_validate_bhs(struct buffer_head **bhs, int nr,
				   struct ocfs2_block_check *bc,
				   struct ocfs2_blockcheck_stats *stats);

/* Debug Initialization */
int ocfs2_blockcheck_stats_debugfs_install(struct ocfs2_blockcheck_stats *stats,
					   struct dentry *parent);
void ocfs2_blockcheck_stats_debugfs_remove(struct ocfs2_blockcheck_stats *stats);


u32 ocfs2_hamming_encode(u32 parity, void *data, unsigned int d,
			 unsigned int nr);
void ocfs2_hamming_fix(void *data, unsigned int d, unsigned int nr,
		       unsigned int fix);

/* Convenience wrappers for a single buffer of data */
extern u32 ocfs2_hamming_encode_block(void *data, unsigned int blocksize);
extern void ocfs2_hamming_fix_block(void *data, unsigned int blocksize,
				    unsigned int fix);
#endif
