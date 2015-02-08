

#ifndef _CFSNC_HEADER_
#define _CFSNC_HEADER_

/* credential cache */
void coda_cache_enter(struct inode *inode, int mask);
void coda_cache_clear_inode(struct inode *);
void coda_cache_clear_all(struct super_block *sb);
int coda_cache_check(struct inode *inode, int mask);

/* for downcalls and attributes and lookups */
void coda_flag_inode_children(struct inode *inode, int flag);

#endif /* _CFSNC_HEADER_ */
