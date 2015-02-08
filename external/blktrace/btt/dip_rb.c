#include <stdio.h>
#include "globals.h"

int rb_insert(struct rb_root *root, struct io *iop)
{
	struct io *__iop;
	struct rb_node *parent = NULL;
	struct rb_node **p = &root->rb_node;
	__u64 __s, s = BIT_START(iop);

	while (*p) {
		parent = *p;
		__iop = rb_entry(parent, struct io, rb_node);
		__s = BIT_START(__iop);

		if (s < __s)
			p = &(*p)->rb_left;
		else if (s > __s)
			p = &(*p)->rb_right;
		else
			return 0;
	}

	rb_link_node(&iop->rb_node, parent, p);
	rb_insert_color(&iop->rb_node, root);
	return 1;
}

struct io *rb_find_sec(struct rb_root *root, __u64 sec)
{
	struct io *__iop;
	struct rb_node *n = root->rb_node;

	while (n) {
		__iop = rb_entry(n, struct io, rb_node);
		if (sec < BIT_START(__iop))
			n = n->rb_left;
		else if (sec >= BIT_END(__iop))
			n = n->rb_right;
		else
			return __iop;
	}

	return NULL;
}

void rb_foreach(struct rb_node *n, struct io *iop,
		      void (*fnc)(struct io *iop, struct io *this),
		      struct list_head *head)
{
	if (n) {
		struct io *this = rb_entry(n, struct io, rb_node);
		__u64 iop_s = BIT_START(iop), iop_e = BIT_END(iop);
		__u64 this_s = BIT_START(this), this_e = BIT_END(this);

		if ((iop_s <= this_s) && (this_e <= iop_e)) {
			if (fnc) fnc(iop, this);
			if (head)
				list_add_tail(&this->f_head, head);
		}
		if (iop_s < this_s)
			rb_foreach(n->rb_left, iop, fnc, head);
		if (this_e < iop_e)
			rb_foreach(n->rb_right, iop, fnc, head);
	}
}
