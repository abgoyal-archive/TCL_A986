

#ifndef __BTRFS_COMPRESSION_
#define __BTRFS_COMPRESSION_

int btrfs_zlib_decompress(unsigned char *data_in,
			  struct page *dest_page,
			  unsigned long start_byte,
			  size_t srclen, size_t destlen);
int btrfs_zlib_compress_pages(struct address_space *mapping,
			      u64 start, unsigned long len,
			      struct page **pages,
			      unsigned long nr_dest_pages,
			      unsigned long *out_pages,
			      unsigned long *total_in,
			      unsigned long *total_out,
			      unsigned long max_out);
int btrfs_zlib_decompress_biovec(struct page **pages_in,
			      u64 disk_start,
			      struct bio_vec *bvec,
			      int vcnt,
			      size_t srclen);
void btrfs_zlib_exit(void);
int btrfs_submit_compressed_write(struct inode *inode, u64 start,
				  unsigned long len, u64 disk_start,
				  unsigned long compressed_len,
				  struct page **compressed_pages,
				  unsigned long nr_pages);
int btrfs_submit_compressed_read(struct inode *inode, struct bio *bio,
				 int mirror_num, unsigned long bio_flags);
#endif
