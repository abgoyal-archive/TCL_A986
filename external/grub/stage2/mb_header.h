

struct multiboot_header
{
  /* Must be MULTIBOOT_MAGIC - see below.  */
  unsigned magic;
  
  /* Feature flags - see below.  */
  unsigned flags;
  
  /*
   * Checksum
   *
   * The above fields plus this one must equal 0 mod 2^32.
   */
  unsigned checksum;
  
  /* These are only valid if MULTIBOOT_AOUT_KLUDGE is set.  */
  unsigned header_addr;
  unsigned load_addr;
  unsigned load_end_addr;
  unsigned bss_end_addr;
  unsigned entry_addr;

  /* These are only valid if MULTIBOOT_VIDEO_MODE is set.  */
  unsigned mode_type;
  unsigned width;
  unsigned height;
  unsigned depth;
};

#define MULTIBOOT_SEARCH		8192
#define MULTIBOOT_FOUND(addr, len) \
  (! ((addr) & 0x3) \
   && (len) >= 12 \
   && *((int *) (addr)) == MULTIBOOT_MAGIC \
   && ! (*((unsigned *) (addr)) + *((unsigned *) (addr + 4)) \
	 + *((unsigned *) (addr + 8))) \
   && (! (MULTIBOOT_AOUT_KLUDGE & *((int *) (addr + 4))) || (len) >= 32) \
   && (! (MULTIBOOT_VIDEO_MODE & *((int *) (addr + 4))) || (len) >= 48))

/* Magic value identifying the multiboot_header.  */
#define MULTIBOOT_MAGIC			0x1BADB002

#define MULTIBOOT_MUSTKNOW		0x0000FFFF

/* currently unsupported flags...  this is a kind of version number.  */
#define MULTIBOOT_UNSUPPORTED		0x0000FFF8

/* Align all boot modules on i386 page (4KB) boundaries.  */
#define MULTIBOOT_PAGE_ALIGN		0x00000001

/* Must pass memory information to OS.  */
#define MULTIBOOT_MEMORY_INFO		0x00000002

/* Must pass video information to OS.  */
#define MULTIBOOT_VIDEO_MODE		0x00000004

/* This flag indicates the use of the address fields in the header.  */
#define MULTIBOOT_AOUT_KLUDGE		0x00010000
