
#ifndef __XFS_AOPS_H__
#define __XFS_AOPS_H__

extern struct workqueue_struct *xfsdatad_workqueue;
extern struct workqueue_struct *xfsconvertd_workqueue;
extern mempool_t *xfs_ioend_pool;

typedef struct xfs_ioend {
	struct xfs_ioend	*io_list;	/* next ioend in chain */
	unsigned int		io_type;	/* delalloc / unwritten */
	int			io_error;	/* I/O error code */
	atomic_t		io_remaining;	/* hold count */
	struct inode		*io_inode;	/* file being written to */
	struct buffer_head	*io_buffer_head;/* buffer linked list head */
	struct buffer_head	*io_buffer_tail;/* buffer linked list tail */
	size_t			io_size;	/* size of the extent */
	xfs_off_t		io_offset;	/* offset in the file */
	struct work_struct	io_work;	/* xfsdatad work queue */
	struct kiocb		*io_iocb;
	int			io_result;
} xfs_ioend_t;

extern const struct address_space_operations xfs_address_space_operations;
extern int xfs_get_blocks(struct inode *, sector_t, struct buffer_head *, int);

extern void xfs_ioend_init(void);
extern void xfs_ioend_wait(struct xfs_inode *);

extern void xfs_count_page_state(struct page *, int *, int *, int *);

#endif /* __XFS_AOPS_H__ */
