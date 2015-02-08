


#ifndef __CVMX_SCRATCH_H__
#define __CVMX_SCRATCH_H__

#define CVMX_SCRATCH_BASE       (-32768l)	/* 0xffffffffffff8000 */

static inline uint8_t cvmx_scratch_read8(uint64_t address)
{
	return *CASTPTR(volatile uint8_t, CVMX_SCRATCH_BASE + address);
}

static inline uint16_t cvmx_scratch_read16(uint64_t address)
{
	return *CASTPTR(volatile uint16_t, CVMX_SCRATCH_BASE + address);
}

static inline uint32_t cvmx_scratch_read32(uint64_t address)
{
	return *CASTPTR(volatile uint32_t, CVMX_SCRATCH_BASE + address);
}

static inline uint64_t cvmx_scratch_read64(uint64_t address)
{
	return *CASTPTR(volatile uint64_t, CVMX_SCRATCH_BASE + address);
}

static inline void cvmx_scratch_write8(uint64_t address, uint64_t value)
{
	*CASTPTR(volatile uint8_t, CVMX_SCRATCH_BASE + address) =
	    (uint8_t) value;
}

static inline void cvmx_scratch_write16(uint64_t address, uint64_t value)
{
	*CASTPTR(volatile uint16_t, CVMX_SCRATCH_BASE + address) =
	    (uint16_t) value;
}

static inline void cvmx_scratch_write32(uint64_t address, uint64_t value)
{
	*CASTPTR(volatile uint32_t, CVMX_SCRATCH_BASE + address) =
	    (uint32_t) value;
}

static inline void cvmx_scratch_write64(uint64_t address, uint64_t value)
{
	*CASTPTR(volatile uint64_t, CVMX_SCRATCH_BASE + address) = value;
}

#endif /* __CVMX_SCRATCH_H__ */
