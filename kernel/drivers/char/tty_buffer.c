

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/module.h>


void tty_buffer_free_all(struct tty_struct *tty)
{
	struct tty_buffer *thead;
	while ((thead = tty->buf.head) != NULL) {
		tty->buf.head = thead->next;
		kfree(thead);
	}
	while ((thead = tty->buf.free) != NULL) {
		tty->buf.free = thead->next;
		kfree(thead);
	}
	tty->buf.tail = NULL;
	tty->buf.memory_used = 0;
}


static struct tty_buffer *tty_buffer_alloc(struct tty_struct *tty, size_t size)
{
	struct tty_buffer *p;

	if (tty->buf.memory_used + size > 65536)
		return NULL;
	p = kmalloc(sizeof(struct tty_buffer) + 2 * size, GFP_ATOMIC);
	if (p == NULL)
		return NULL;
	p->used = 0;
	p->size = size;
	p->next = NULL;
	p->commit = 0;
	p->read = 0;
	p->char_buf_ptr = (char *)(p->data);
	p->flag_buf_ptr = (unsigned char *)p->char_buf_ptr + size;
	tty->buf.memory_used += size;
	return p;
}


static void tty_buffer_free(struct tty_struct *tty, struct tty_buffer *b)
{
	/* Dumb strategy for now - should keep some stats */
	tty->buf.memory_used -= b->size;
	WARN_ON(tty->buf.memory_used < 0);

	if (b->size >= 512)
		kfree(b);
	else {
		b->next = tty->buf.free;
		tty->buf.free = b;
	}
}


static void __tty_buffer_flush(struct tty_struct *tty)
{
	struct tty_buffer *thead;

	while ((thead = tty->buf.head) != NULL) {
		tty->buf.head = thead->next;
		tty_buffer_free(tty, thead);
	}
	tty->buf.tail = NULL;
}


void tty_buffer_flush(struct tty_struct *tty)
{
	unsigned long flags;
	spin_lock_irqsave(&tty->buf.lock, flags);

	/* If the data is being pushed to the tty layer then we can't
	   process it here. Instead set a flag and the flush_to_ldisc
	   path will process the flush request before it exits */
	if (test_bit(TTY_FLUSHING, &tty->flags)) {
		set_bit(TTY_FLUSHPENDING, &tty->flags);
		spin_unlock_irqrestore(&tty->buf.lock, flags);
		wait_event(tty->read_wait,
				test_bit(TTY_FLUSHPENDING, &tty->flags) == 0);
		return;
	} else
		__tty_buffer_flush(tty);
	spin_unlock_irqrestore(&tty->buf.lock, flags);
}


static struct tty_buffer *tty_buffer_find(struct tty_struct *tty, size_t size)
{
	struct tty_buffer **tbh = &tty->buf.free;
	while ((*tbh) != NULL) {
		struct tty_buffer *t = *tbh;
		if (t->size >= size) {
			*tbh = t->next;
			t->next = NULL;
			t->used = 0;
			t->commit = 0;
			t->read = 0;
			tty->buf.memory_used += t->size;
			return t;
		}
		tbh = &((*tbh)->next);
	}
	/* Round the buffer size out */
	size = (size + 0xFF) & ~0xFF;
	return tty_buffer_alloc(tty, size);
	/* Should possibly check if this fails for the largest buffer we
	   have queued and recycle that ? */
}

int tty_buffer_request_room(struct tty_struct *tty, size_t size)
{
	struct tty_buffer *b, *n;
	int left;
	unsigned long flags;

	spin_lock_irqsave(&tty->buf.lock, flags);

	/* OPTIMISATION: We could keep a per tty "zero" sized buffer to
	   remove this conditional if its worth it. This would be invisible
	   to the callers */
	if ((b = tty->buf.tail) != NULL)
		left = b->size - b->used;
	else
		left = 0;

	if (left < size) {
		/* This is the slow path - looking for new buffers to use */
		if ((n = tty_buffer_find(tty, size)) != NULL) {
			if (b != NULL) {
				b->next = n;
				b->commit = b->used;
			} else
				tty->buf.head = n;
			tty->buf.tail = n;
		} else
			size = left;
	}

	spin_unlock_irqrestore(&tty->buf.lock, flags);
	return size;
}
EXPORT_SYMBOL_GPL(tty_buffer_request_room);


int tty_insert_flip_string_fixed_flag(struct tty_struct *tty,
		const unsigned char *chars, char flag, size_t size)
{
	int copied = 0;
	do {
		int goal = min_t(size_t, size - copied, TTY_BUFFER_PAGE);
		int space = tty_buffer_request_room(tty, goal);
		struct tty_buffer *tb = tty->buf.tail;
		/* If there is no space then tb may be NULL */
		if (unlikely(space == 0))
			break;
		memcpy(tb->char_buf_ptr + tb->used, chars, space);
		memset(tb->flag_buf_ptr + tb->used, flag, space);
		tb->used += space;
		copied += space;
		chars += space;
		/* There is a small chance that we need to split the data over
		   several buffers. If this is the case we must loop */
	} while (unlikely(size > copied));
	return copied;
}
EXPORT_SYMBOL(tty_insert_flip_string_fixed_flag);


int tty_insert_flip_string_flags(struct tty_struct *tty,
		const unsigned char *chars, const char *flags, size_t size)
{
	int copied = 0;
	do {
		int goal = min_t(size_t, size - copied, TTY_BUFFER_PAGE);
		int space = tty_buffer_request_room(tty, goal);
		struct tty_buffer *tb = tty->buf.tail;
		/* If there is no space then tb may be NULL */
		if (unlikely(space == 0))
			break;
		memcpy(tb->char_buf_ptr + tb->used, chars, space);
		memcpy(tb->flag_buf_ptr + tb->used, flags, space);
		tb->used += space;
		copied += space;
		chars += space;
		flags += space;
		/* There is a small chance that we need to split the data over
		   several buffers. If this is the case we must loop */
	} while (unlikely(size > copied));
	return copied;
}
EXPORT_SYMBOL(tty_insert_flip_string_flags);


void tty_schedule_flip(struct tty_struct *tty)
{
	unsigned long flags;
	spin_lock_irqsave(&tty->buf.lock, flags);
	if (tty->buf.tail != NULL)
		tty->buf.tail->commit = tty->buf.tail->used;
	spin_unlock_irqrestore(&tty->buf.lock, flags);
	schedule_delayed_work(&tty->buf.work, 1);
}
EXPORT_SYMBOL(tty_schedule_flip);


int tty_prepare_flip_string(struct tty_struct *tty, unsigned char **chars,
								size_t size)
{
	int space = tty_buffer_request_room(tty, size);
	if (likely(space)) {
		struct tty_buffer *tb = tty->buf.tail;
		*chars = tb->char_buf_ptr + tb->used;
		memset(tb->flag_buf_ptr + tb->used, TTY_NORMAL, space);
		tb->used += space;
	}
	return space;
}
EXPORT_SYMBOL_GPL(tty_prepare_flip_string);


int tty_prepare_flip_string_flags(struct tty_struct *tty,
			unsigned char **chars, char **flags, size_t size)
{
	int space = tty_buffer_request_room(tty, size);
	if (likely(space)) {
		struct tty_buffer *tb = tty->buf.tail;
		*chars = tb->char_buf_ptr + tb->used;
		*flags = tb->flag_buf_ptr + tb->used;
		tb->used += space;
	}
	return space;
}
EXPORT_SYMBOL_GPL(tty_prepare_flip_string_flags);




static void flush_to_ldisc(struct work_struct *work)
{
	struct tty_struct *tty =
		container_of(work, struct tty_struct, buf.work.work);
	unsigned long 	flags;
	struct tty_ldisc *disc;

	disc = tty_ldisc_ref(tty);
	if (disc == NULL)	/*  !TTY_LDISC */
		return;

	spin_lock_irqsave(&tty->buf.lock, flags);

	if (!test_and_set_bit(TTY_FLUSHING, &tty->flags)) {
		struct tty_buffer *head;
		while ((head = tty->buf.head) != NULL) {
			int count;
			char *char_buf;
			unsigned char *flag_buf;

			count = head->commit - head->read;
			if (!count) {
				if (head->next == NULL)
					break;
				tty->buf.head = head->next;
				tty_buffer_free(tty, head);
				continue;
			}
			/* Ldisc or user is trying to flush the buffers
			   we are feeding to the ldisc, stop feeding the
			   line discipline as we want to empty the queue */
			if (test_bit(TTY_FLUSHPENDING, &tty->flags))
				break;
			if (!tty->receive_room) {
				schedule_delayed_work(&tty->buf.work, 1);
				break;
			}
			if (count > tty->receive_room)
				count = tty->receive_room;
			char_buf = head->char_buf_ptr + head->read;
			flag_buf = head->flag_buf_ptr + head->read;
			head->read += count;
			spin_unlock_irqrestore(&tty->buf.lock, flags);
			disc->ops->receive_buf(tty, char_buf,
							flag_buf, count);
			spin_lock_irqsave(&tty->buf.lock, flags);
		}
		clear_bit(TTY_FLUSHING, &tty->flags);
	}

	/* We may have a deferred request to flush the input buffer,
	   if so pull the chain under the lock and empty the queue */
	if (test_bit(TTY_FLUSHPENDING, &tty->flags)) {
		__tty_buffer_flush(tty);
		clear_bit(TTY_FLUSHPENDING, &tty->flags);
		wake_up(&tty->read_wait);
	}
	spin_unlock_irqrestore(&tty->buf.lock, flags);

	tty_ldisc_deref(disc);
}

void tty_flush_to_ldisc(struct tty_struct *tty)
{
	flush_delayed_work(&tty->buf.work);
}


void tty_flip_buffer_push(struct tty_struct *tty)
{
	unsigned long flags;
	spin_lock_irqsave(&tty->buf.lock, flags);
	if (tty->buf.tail != NULL)
		tty->buf.tail->commit = tty->buf.tail->used;
	spin_unlock_irqrestore(&tty->buf.lock, flags);

	if (tty->low_latency)
		flush_to_ldisc(&tty->buf.work.work);
	else
		schedule_delayed_work(&tty->buf.work, 1);
}
EXPORT_SYMBOL(tty_flip_buffer_push);


void tty_buffer_init(struct tty_struct *tty)
{
	spin_lock_init(&tty->buf.lock);
	tty->buf.head = NULL;
	tty->buf.tail = NULL;
	tty->buf.free = NULL;
	tty->buf.memory_used = 0;
	INIT_DELAYED_WORK(&tty->buf.work, flush_to_ldisc);
}

