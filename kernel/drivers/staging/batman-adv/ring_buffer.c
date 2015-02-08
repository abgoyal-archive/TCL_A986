

#include "main.h"
#include "ring_buffer.h"

void ring_buffer_set(uint8_t lq_recv[], uint8_t *lq_index, uint8_t value)
{
	lq_recv[*lq_index] = value;
	*lq_index = (*lq_index + 1) % TQ_GLOBAL_WINDOW_SIZE;
}

uint8_t ring_buffer_avg(uint8_t lq_recv[])
{
	uint8_t *ptr;
	uint16_t count = 0, i = 0, sum = 0;

	ptr = lq_recv;

	while (i < TQ_GLOBAL_WINDOW_SIZE) {
		if (*ptr != 0) {
			count++;
			sum += *ptr;
		}

		i++;
		ptr++;
	}

	if (count == 0)
		return 0;

	return (uint8_t)(sum / count);
}
