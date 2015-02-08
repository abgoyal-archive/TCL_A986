

#ifndef IB_UMEM_H
#define IB_UMEM_H

#include <linux/list.h>
#include <linux/scatterlist.h>
#include <linux/workqueue.h>

struct ib_ucontext;

struct ib_umem {
	struct ib_ucontext     *context;
	size_t			length;
	int			offset;
	int			page_size;
	int                     writable;
	int                     hugetlb;
	struct list_head	chunk_list;
	struct work_struct	work;
	struct mm_struct       *mm;
	unsigned long		diff;
};

struct ib_umem_chunk {
	struct list_head	list;
	int                     nents;
	int                     nmap;
	struct scatterlist      page_list[0];
};

#ifdef CONFIG_INFINIBAND_USER_MEM

struct ib_umem *ib_umem_get(struct ib_ucontext *context, unsigned long addr,
			    size_t size, int access, int dmasync);
void ib_umem_release(struct ib_umem *umem);
int ib_umem_page_count(struct ib_umem *umem);

#else /* CONFIG_INFINIBAND_USER_MEM */

#include <linux/err.h>

static inline struct ib_umem *ib_umem_get(struct ib_ucontext *context,
					  unsigned long addr, size_t size,
					  int access, int dmasync) {
	return ERR_PTR(-EINVAL);
}
static inline void ib_umem_release(struct ib_umem *umem) { }
static inline int ib_umem_page_count(struct ib_umem *umem) { return 0; }

#endif /* CONFIG_INFINIBAND_USER_MEM */

#endif /* IB_UMEM_H */
