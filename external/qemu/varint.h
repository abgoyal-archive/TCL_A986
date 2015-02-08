
#include <inttypes.h>

extern char *varint_encode(uint64_t value, char *buf);
extern char *varint_encode_signed(int64_t value, char *buf);
