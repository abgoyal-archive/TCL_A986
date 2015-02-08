

#include <stdio.h>

#include "qemu.h"
#include "qemu-common.h"
#include "ioport.h"

void cpu_outb(pio_addr_t addr, uint8_t val)
{
    fprintf(stderr, "outb: port=0x%04"FMT_pioaddr", data=%02"PRIx8"\n",
            addr, val);
}

void cpu_outw(pio_addr_t addr, uint16_t val)
{
    fprintf(stderr, "outw: port=0x%04"FMT_pioaddr", data=%04"PRIx16"\n",
            addr, val);
}

void cpu_outl(pio_addr_t addr, uint32_t val)
{
    fprintf(stderr, "outl: port=0x%04"FMT_pioaddr", data=%08"PRIx32"\n",
            addr, val);
}

uint8_t cpu_inb(pio_addr_t addr)
{
    fprintf(stderr, "inb: port=0x%04"FMT_pioaddr"\n", addr);
    return 0;
}

uint16_t cpu_inw(pio_addr_t addr)
{
    fprintf(stderr, "inw: port=0x%04"FMT_pioaddr"\n", addr);
    return 0;
}

uint32_t cpu_inl(pio_addr_t addr)
{
    fprintf(stderr, "inl: port=0x%04"FMT_pioaddr"\n", addr);
    return 0;
}
