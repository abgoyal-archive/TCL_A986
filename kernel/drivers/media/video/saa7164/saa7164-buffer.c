

#include <linux/slab.h>

#include "saa7164.h"


struct saa7164_buffer *saa7164_buffer_alloc(struct saa7164_tsport *port,
	u32 len)
{
	struct saa7164_buffer *buf = 0;
	struct saa7164_dev *dev = port->dev;
	int i;

	if ((len == 0) || (len >= 65536) || (len % sizeof(u64))) {
		log_warn("%s() SAA_ERR_BAD_PARAMETER\n", __func__);
		goto ret;
	}

	buf = kzalloc(sizeof(struct saa7164_buffer), GFP_KERNEL);
	if (buf == NULL) {
		log_warn("%s() SAA_ERR_NO_RESOURCES\n", __func__);
		goto ret;
	}

	buf->port = port;
	buf->flags = SAA7164_BUFFER_FREE;
	/* TODO: arg len is being ignored */
	buf->pci_size = SAA7164_PT_ENTRIES * 0x1000;
	buf->pt_size = (SAA7164_PT_ENTRIES * sizeof(u64)) + 0x1000;

	/* Allocate contiguous memory */
	buf->cpu = pci_alloc_consistent(port->dev->pci, buf->pci_size,
		&buf->dma);
	if (!buf->cpu)
		goto fail1;

	buf->pt_cpu = pci_alloc_consistent(port->dev->pci, buf->pt_size,
		&buf->pt_dma);
	if (!buf->pt_cpu)
		goto fail2;

	/* init the buffers to a known pattern, easier during debugging */
	memset(buf->cpu, 0xff, buf->pci_size);
	memset(buf->pt_cpu, 0xff, buf->pt_size);

	dprintk(DBGLVL_BUF, "%s()   allocated buffer @ 0x%p\n", __func__, buf);
	dprintk(DBGLVL_BUF, "  pci_cpu @ 0x%p    dma @ 0x%08lx len = 0x%x\n",
		buf->cpu, (long)buf->dma, buf->pci_size);
	dprintk(DBGLVL_BUF, "   pt_cpu @ 0x%p pt_dma @ 0x%08lx len = 0x%x\n",
		buf->pt_cpu, (long)buf->pt_dma, buf->pt_size);

	/* Format the Page Table Entries to point into the data buffer */
	for (i = 0 ; i < SAA7164_PT_ENTRIES; i++) {

		*(buf->pt_cpu + i) = buf->dma + (i * 0x1000); /* TODO */

	}

	goto ret;

fail2:
	pci_free_consistent(port->dev->pci, buf->pci_size, buf->cpu, buf->dma);
fail1:
	kfree(buf);

	buf = 0;
ret:
	return buf;
}

int saa7164_buffer_dealloc(struct saa7164_tsport *port,
	struct saa7164_buffer *buf)
{
	struct saa7164_dev *dev = port->dev;

	if ((buf == 0) || (port == 0))
		return SAA_ERR_BAD_PARAMETER;

	dprintk(DBGLVL_BUF, "%s() deallocating buffer @ 0x%p\n", __func__, buf);

	if (buf->flags != SAA7164_BUFFER_FREE)
		log_warn(" freeing a non-free buffer\n");

	pci_free_consistent(port->dev->pci, buf->pci_size, buf->cpu, buf->dma);
	pci_free_consistent(port->dev->pci, buf->pt_size, buf->pt_cpu,
		buf->pt_dma);

	kfree(buf);

	return SAA_OK;
}

