

#include <linux/highmem.h>


#define BTRFS_SETGET_FUNCS(name, type, member, bits)			\
u##bits btrfs_##name(struct extent_buffer *eb, type *s);		\
void btrfs_set_##name(struct extent_buffer *eb, type *s, u##bits val);	\
u##bits btrfs_##name(struct extent_buffer *eb,				\
				   type *s)				\
{									\
	unsigned long part_offset = (unsigned long)s;			\
	unsigned long offset = part_offset + offsetof(type, member);	\
	type *p;							\
	/* ugly, but we want the fast path here */			\
	if (eb->map_token && offset >= eb->map_start &&			\
	    offset + sizeof(((type *)0)->member) <= eb->map_start +	\
	    eb->map_len) {						\
		p = (type *)(eb->kaddr + part_offset - eb->map_start);	\
		return le##bits##_to_cpu(p->member);			\
	}								\
	{								\
		int err;						\
		char *map_token;					\
		char *kaddr;						\
		int unmap_on_exit = (eb->map_token == NULL);		\
		unsigned long map_start;				\
		unsigned long map_len;					\
		u##bits res;						\
		err = map_extent_buffer(eb, offset,			\
				sizeof(((type *)0)->member),		\
				&map_token, &kaddr,			\
				&map_start, &map_len, KM_USER1);	\
		if (err) {						\
			__le##bits leres;				\
			read_eb_member(eb, s, type, member, &leres);	\
			return le##bits##_to_cpu(leres);		\
		}							\
		p = (type *)(kaddr + part_offset - map_start);		\
		res = le##bits##_to_cpu(p->member);			\
		if (unmap_on_exit)					\
			unmap_extent_buffer(eb, map_token, KM_USER1);	\
		return res;						\
	}								\
}									\
void btrfs_set_##name(struct extent_buffer *eb,				\
				    type *s, u##bits val)		\
{									\
	unsigned long part_offset = (unsigned long)s;			\
	unsigned long offset = part_offset + offsetof(type, member);	\
	type *p;							\
	/* ugly, but we want the fast path here */			\
	if (eb->map_token && offset >= eb->map_start &&			\
	    offset + sizeof(((type *)0)->member) <= eb->map_start +	\
	    eb->map_len) {						\
		p = (type *)(eb->kaddr + part_offset - eb->map_start);	\
		p->member = cpu_to_le##bits(val);			\
		return;							\
	}								\
	{								\
		int err;						\
		char *map_token;					\
		char *kaddr;						\
		int unmap_on_exit = (eb->map_token == NULL);		\
		unsigned long map_start;				\
		unsigned long map_len;					\
		err = map_extent_buffer(eb, offset,			\
				sizeof(((type *)0)->member),		\
				&map_token, &kaddr,			\
				&map_start, &map_len, KM_USER1);	\
		if (err) {						\
			__le##bits val2;				\
			val2 = cpu_to_le##bits(val);			\
			write_eb_member(eb, s, type, member, &val2);	\
			return;						\
		}							\
		p = (type *)(kaddr + part_offset - map_start);		\
		p->member = cpu_to_le##bits(val);			\
		if (unmap_on_exit)					\
			unmap_extent_buffer(eb, map_token, KM_USER1);	\
	}								\
}

#include "ctree.h"

void btrfs_node_key(struct extent_buffer *eb,
		    struct btrfs_disk_key *disk_key, int nr)
{
	unsigned long ptr = btrfs_node_key_ptr_offset(nr);
	if (eb->map_token && ptr >= eb->map_start &&
	    ptr + sizeof(*disk_key) <= eb->map_start + eb->map_len) {
		memcpy(disk_key, eb->kaddr + ptr - eb->map_start,
			sizeof(*disk_key));
		return;
	} else if (eb->map_token) {
		unmap_extent_buffer(eb, eb->map_token, KM_USER1);
		eb->map_token = NULL;
	}
	read_eb_member(eb, (struct btrfs_key_ptr *)ptr,
		       struct btrfs_key_ptr, key, disk_key);
}
