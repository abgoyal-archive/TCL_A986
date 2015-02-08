

#ifndef _IIO_RING_SW_H_
#define _IIO_RING_SW_H_
/* NEEDS COMMENTS */


#include "iio.h"
#include "ring_generic.h"

#if defined CONFIG_IIO_SW_RING || defined CONFIG_IIO_SW_RING_MODULE

int iio_create_sw_rb(struct iio_ring_buffer **r);

int iio_init_sw_rb(struct iio_ring_buffer *r, struct iio_dev *indio_dev);

void iio_exit_sw_rb(struct iio_ring_buffer *r);

void iio_free_sw_rb(struct iio_ring_buffer *r);

void iio_mark_sw_rb_in_use(struct iio_ring_buffer *r);

void iio_unmark_sw_rb_in_use(struct iio_ring_buffer *r);

int iio_read_last_from_sw_rb(struct iio_ring_buffer *r, u8 *data);

int iio_store_to_sw_rb(struct iio_ring_buffer *r, u8 *data, s64 timestamp);

int iio_rip_sw_rb(struct iio_ring_buffer *r,
		  size_t count,
		  u8 **data,
		  int *dead_offset);

int iio_request_update_sw_rb(struct iio_ring_buffer *r);

int iio_mark_update_needed_sw_rb(struct iio_ring_buffer *r);


int iio_get_bpd_sw_rb(struct iio_ring_buffer *r);

int iio_set_bpd_sw_rb(struct iio_ring_buffer *r, size_t bpd);

int iio_get_length_sw_rb(struct iio_ring_buffer *r);

int iio_set_length_sw_rb(struct iio_ring_buffer *r, int length);

static inline void iio_ring_sw_register_funcs(struct iio_ring_access_funcs *ra)
{
	ra->mark_in_use = &iio_mark_sw_rb_in_use;
	ra->unmark_in_use = &iio_unmark_sw_rb_in_use;

	ra->store_to = &iio_store_to_sw_rb;
	ra->read_last = &iio_read_last_from_sw_rb;
	ra->rip_lots = &iio_rip_sw_rb;

	ra->mark_param_change = &iio_mark_update_needed_sw_rb;
	ra->request_update = &iio_request_update_sw_rb;

	ra->get_bpd = &iio_get_bpd_sw_rb;
	ra->set_bpd = &iio_set_bpd_sw_rb;

	ra->get_length = &iio_get_length_sw_rb;
	ra->set_length = &iio_set_length_sw_rb;
};


struct iio_sw_ring_buffer {
	struct iio_ring_buffer  buf;
	unsigned char		*data;
	unsigned char		*read_p;
	unsigned char		*write_p;
	unsigned char		*last_written_p;
	/* used to act as a point at which to signal an event */
	unsigned char		*half_p;
	int			use_count;
	int			update_needed;
	spinlock_t		use_lock;
};

#define iio_to_sw_ring(r) container_of(r, struct iio_sw_ring_buffer, buf)

struct iio_ring_buffer *iio_sw_rb_allocate(struct iio_dev *indio_dev);
void iio_sw_rb_free(struct iio_ring_buffer *ring);



#else /* CONFIG_IIO_RING_BUFFER*/
static inline void iio_ring_sw_register_funcs(struct iio_ring_access_funcs *ra)
{};
#endif /* !CONFIG_IIO_RING_BUFFER */
#endif /* _IIO_RING_SW_H_ */
