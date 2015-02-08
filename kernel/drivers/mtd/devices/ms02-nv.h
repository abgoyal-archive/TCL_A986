

#include <linux/ioport.h>
#include <linux/mtd/mtd.h>


/* MS02-NV iomem register offsets. */
#define MS02NV_CSR		0x400000	/* control & status register */

/* MS02-NV CSR status bits. */
#define MS02NV_CSR_BATT_OK	0x01		/* battery OK */
#define MS02NV_CSR_BATT_OFF	0x02		/* battery disabled */


/* MS02-NV memory offsets. */
#define MS02NV_DIAG		0x0003f8	/* diagnostic status */
#define MS02NV_MAGIC		0x0003fc	/* MS02-NV magic ID */
#define MS02NV_VALID		0x000400	/* valid data magic ID */
#define MS02NV_RAM		0x001000	/* user-exposed RAM start */

/* MS02-NV diagnostic status bits. */
#define MS02NV_DIAG_TEST	0x01		/* SRAM test done (?) */
#define MS02NV_DIAG_RO		0x02		/* SRAM r/o test done */
#define MS02NV_DIAG_RW		0x04		/* SRAM r/w test done */
#define MS02NV_DIAG_FAIL	0x08		/* SRAM test failed */
#define MS02NV_DIAG_SIZE_MASK	0xf0		/* SRAM size mask */
#define MS02NV_DIAG_SIZE_SHIFT	0x10		/* SRAM size shift (left) */

/* MS02-NV general constants. */
#define MS02NV_ID		0x03021966	/* MS02-NV magic ID value */
#define MS02NV_VALID_ID		0xbd100248	/* valid data magic ID value */
#define MS02NV_SLOT_SIZE	0x800000	/* size of the address space
						   decoded by the module */


typedef volatile u32 ms02nv_uint;

struct ms02nv_private {
	struct mtd_info *next;
	struct {
		struct resource *module;
		struct resource *diag_ram;
		struct resource *user_ram;
		struct resource *csr;
	} resource;
	u_char *addr;
	size_t size;
	u_char *uaddr;
};
