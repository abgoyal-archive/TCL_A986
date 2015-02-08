

#ifndef _DRM_BUFFER_H_
#define _DRM_BUFFER_H_

#include "drmP.h"

struct drm_buffer {
	int iterator;
	int size;
	char *data[];
};


static inline int drm_buffer_page(struct drm_buffer *buf)
{
	return buf->iterator / PAGE_SIZE;
}
static inline int drm_buffer_index(struct drm_buffer *buf)
{
	return buf->iterator & (PAGE_SIZE - 1);
}
static inline int drm_buffer_unprocessed(struct drm_buffer *buf)
{
	return buf->size - buf->iterator;
}

static inline void drm_buffer_advance(struct drm_buffer *buf, int bytes)
{
	buf->iterator += bytes;
}

extern int drm_buffer_alloc(struct drm_buffer **buf, int size);

extern int drm_buffer_copy_from_user(struct drm_buffer *buf,
		void __user *user_data, int size);

extern void drm_buffer_free(struct drm_buffer *buf);

extern void *drm_buffer_read_object(struct drm_buffer *buf,
		int objsize, void *stack_obj);

static inline void *drm_buffer_pointer_to_dword(struct drm_buffer *buffer,
		int offset)
{
	int iter = buffer->iterator + offset * 4;
	return &buffer->data[iter / PAGE_SIZE][iter & (PAGE_SIZE - 1)];
}
static inline void *drm_buffer_pointer_to_byte(struct drm_buffer *buffer,
		int offset)
{
	int iter = buffer->iterator + offset;
	return &buffer->data[iter / PAGE_SIZE][iter & (PAGE_SIZE - 1)];
}

#endif
