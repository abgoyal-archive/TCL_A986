

#ifndef QEMU_MODULE_H
#define QEMU_MODULE_H

/* This should not be used directly.  Use block_init etc. instead.  */
#define module_init(function, type)                                         \
static void __attribute__((constructor)) do_qemu_init_ ## function(void) {  \
    register_module_init(function, type);                                   \
}

typedef enum {
    MODULE_INIT_BLOCK,
    MODULE_INIT_DEVICE,
    MODULE_INIT_MACHINE,
    MODULE_INIT_MAX
} module_init_type;

#define block_init(function) module_init(function, MODULE_INIT_BLOCK)
#define device_init(function) module_init(function, MODULE_INIT_DEVICE)
#define machine_init(function) module_init(function, MODULE_INIT_MACHINE)

void register_module_init(void (*fn)(void), module_init_type type);

void module_call_init(module_init_type type);

#endif
