

struct map_desc;
struct sys_timer;

/* core initialisation functions */

extern void nuc900_init_irq(void);
extern struct sys_timer nuc900_timer;

/* extern file from nuc950.c */

extern void nuc950_board_init(void);
extern void nuc950_init_clocks(void);
extern void nuc950_map_io(void);
