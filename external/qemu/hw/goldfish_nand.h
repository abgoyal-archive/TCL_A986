
#ifndef NAND_DEVICE_H
#define NAND_DEVICE_H

void nand_dev_init(uint32_t base);
void nand_add_dev(const char *arg);

typedef struct {
    uint64_t     limit;
    uint64_t     counter;
    int          pid;
    int          signal;
} nand_threshold;

extern nand_threshold   android_nand_read_threshold;
extern nand_threshold   android_nand_write_threshold;

#endif
