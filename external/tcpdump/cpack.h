

#ifndef _CPACK_H
#define _CPACK_H

struct cpack_state {
	u_int8_t					*c_buf;
	u_int8_t					*c_next;
	size_t						 c_len;
};

int cpack_init(struct cpack_state *, u_int8_t *, size_t);

int cpack_uint8(struct cpack_state *, u_int8_t *);
int cpack_uint16(struct cpack_state *, u_int16_t *);
int cpack_uint32(struct cpack_state *, u_int32_t *);
int cpack_uint64(struct cpack_state *, u_int64_t *);

#define cpack_int8(__s, __p)	cpack_uint8((__s),  (u_int8_t*)(__p))
#define cpack_int16(__s, __p)	cpack_uint16((__s), (u_int16_t*)(__p))
#define cpack_int32(__s, __p)	cpack_uint32((__s), (u_int32_t*)(__p))
#define cpack_int64(__s, __p)	cpack_uint64((__s), (u_int64_t*)(__p))

#endif /* _CPACK_H */
