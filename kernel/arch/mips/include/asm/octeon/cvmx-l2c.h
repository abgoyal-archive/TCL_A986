


#ifndef __CVMX_L2C_H__
#define __CVMX_L2C_H__

/* Deprecated macro, use function */
#define CVMX_L2_ASSOC     cvmx_l2c_get_num_assoc()

/* Deprecated macro, use function */
#define CVMX_L2_SET_BITS  cvmx_l2c_get_set_bits()

/* Deprecated macro, use function */
#define CVMX_L2_SETS      cvmx_l2c_get_num_sets()

#define CVMX_L2C_IDX_ADDR_SHIFT 7  /* based on 128 byte cache line size */
#define CVMX_L2C_IDX_MASK       (cvmx_l2c_get_num_sets() - 1)

/* Defines for index aliasing computations */
#define CVMX_L2C_TAG_ADDR_ALIAS_SHIFT \
	(CVMX_L2C_IDX_ADDR_SHIFT + cvmx_l2c_get_set_bits())

#define CVMX_L2C_ALIAS_MASK \
	(CVMX_L2C_IDX_MASK << CVMX_L2C_TAG_ADDR_ALIAS_SHIFT)

union cvmx_l2c_tag {
	uint64_t u64;
	struct {
		uint64_t reserved:28;
		uint64_t V:1;	/* Line valid */
		uint64_t D:1;	/* Line dirty */
		uint64_t L:1;	/* Line locked */
		uint64_t U:1;	/* Use, LRU eviction */
		uint64_t addr:32;	/* Phys mem (not all bits valid) */
	} s;
};

  /* L2C Performance Counter events. */
enum cvmx_l2c_event {
	CVMX_L2C_EVENT_CYCLES = 0,
	CVMX_L2C_EVENT_INSTRUCTION_MISS = 1,
	CVMX_L2C_EVENT_INSTRUCTION_HIT = 2,
	CVMX_L2C_EVENT_DATA_MISS = 3,
	CVMX_L2C_EVENT_DATA_HIT = 4,
	CVMX_L2C_EVENT_MISS = 5,
	CVMX_L2C_EVENT_HIT = 6,
	CVMX_L2C_EVENT_VICTIM_HIT = 7,
	CVMX_L2C_EVENT_INDEX_CONFLICT = 8,
	CVMX_L2C_EVENT_TAG_PROBE = 9,
	CVMX_L2C_EVENT_TAG_UPDATE = 10,
	CVMX_L2C_EVENT_TAG_COMPLETE = 11,
	CVMX_L2C_EVENT_TAG_DIRTY = 12,
	CVMX_L2C_EVENT_DATA_STORE_NOP = 13,
	CVMX_L2C_EVENT_DATA_STORE_READ = 14,
	CVMX_L2C_EVENT_DATA_STORE_WRITE = 15,
	CVMX_L2C_EVENT_FILL_DATA_VALID = 16,
	CVMX_L2C_EVENT_WRITE_REQUEST = 17,
	CVMX_L2C_EVENT_READ_REQUEST = 18,
	CVMX_L2C_EVENT_WRITE_DATA_VALID = 19,
	CVMX_L2C_EVENT_XMC_NOP = 20,
	CVMX_L2C_EVENT_XMC_LDT = 21,
	CVMX_L2C_EVENT_XMC_LDI = 22,
	CVMX_L2C_EVENT_XMC_LDD = 23,
	CVMX_L2C_EVENT_XMC_STF = 24,
	CVMX_L2C_EVENT_XMC_STT = 25,
	CVMX_L2C_EVENT_XMC_STP = 26,
	CVMX_L2C_EVENT_XMC_STC = 27,
	CVMX_L2C_EVENT_XMC_DWB = 28,
	CVMX_L2C_EVENT_XMC_PL2 = 29,
	CVMX_L2C_EVENT_XMC_PSL1 = 30,
	CVMX_L2C_EVENT_XMC_IOBLD = 31,
	CVMX_L2C_EVENT_XMC_IOBST = 32,
	CVMX_L2C_EVENT_XMC_IOBDMA = 33,
	CVMX_L2C_EVENT_XMC_IOBRSP = 34,
	CVMX_L2C_EVENT_XMC_BUS_VALID = 35,
	CVMX_L2C_EVENT_XMC_MEM_DATA = 36,
	CVMX_L2C_EVENT_XMC_REFL_DATA = 37,
	CVMX_L2C_EVENT_XMC_IOBRSP_DATA = 38,
	CVMX_L2C_EVENT_RSC_NOP = 39,
	CVMX_L2C_EVENT_RSC_STDN = 40,
	CVMX_L2C_EVENT_RSC_FILL = 41,
	CVMX_L2C_EVENT_RSC_REFL = 42,
	CVMX_L2C_EVENT_RSC_STIN = 43,
	CVMX_L2C_EVENT_RSC_SCIN = 44,
	CVMX_L2C_EVENT_RSC_SCFL = 45,
	CVMX_L2C_EVENT_RSC_SCDN = 46,
	CVMX_L2C_EVENT_RSC_DATA_VALID = 47,
	CVMX_L2C_EVENT_RSC_VALID_FILL = 48,
	CVMX_L2C_EVENT_RSC_VALID_STRSP = 49,
	CVMX_L2C_EVENT_RSC_VALID_REFL = 50,
	CVMX_L2C_EVENT_LRF_REQ = 51,
	CVMX_L2C_EVENT_DT_RD_ALLOC = 52,
	CVMX_L2C_EVENT_DT_WR_INVAL = 53
};

void cvmx_l2c_config_perf(uint32_t counter,
			  enum cvmx_l2c_event event, uint32_t clear_on_read);
uint64_t cvmx_l2c_read_perf(uint32_t counter);

int cvmx_l2c_get_core_way_partition(uint32_t core);

int cvmx_l2c_set_core_way_partition(uint32_t core, uint32_t mask);

int cvmx_l2c_get_hw_way_partition(void);

int cvmx_l2c_set_hw_way_partition(uint32_t mask);

int cvmx_l2c_lock_line(uint64_t addr);

int cvmx_l2c_lock_mem_region(uint64_t start, uint64_t len);

int cvmx_l2c_unlock_line(uint64_t address);

int cvmx_l2c_unlock_mem_region(uint64_t start, uint64_t len);

union cvmx_l2c_tag cvmx_l2c_get_tag(uint32_t association, uint32_t index);

/* Wrapper around deprecated old function name */
static inline union cvmx_l2c_tag cvmx_get_l2c_tag(uint32_t association,
					      uint32_t index)
{
	return cvmx_l2c_get_tag(association, index);
}

uint32_t cvmx_l2c_address_to_index(uint64_t addr);

void cvmx_l2c_flush(void);

int cvmx_l2c_get_cache_size_bytes(void);

int cvmx_l2c_get_num_sets(void);

int cvmx_l2c_get_set_bits(void);
int cvmx_l2c_get_num_assoc(void);

void cvmx_l2c_flush_line(uint32_t assoc, uint32_t index);

#endif /* __CVMX_L2C_H__ */
