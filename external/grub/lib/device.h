/* device.h - Define macros and declare prototypes for device.c */

#ifndef DEVICE_MAP_HEADER
#define DEVICE_MAP_HEADER	1

/* The maximum number of BIOS disks.  */
#define NUM_DISKS	256

/* Simulated disk sizes. */
#define DEFAULT_FD_CYLINDERS	80
#define DEFAULT_FD_HEADS	2
#define DEFAULT_FD_SECTORS	18
#define DEFAULT_HD_CYLINDERS	620
#define DEFAULT_HD_HEADS	128
#define DEFAULT_HD_SECTORS	63

/* Function prototypes.  */
extern void get_drive_geometry (struct geometry *geom, char **map, int drive);
extern int check_device (const char *device);
extern int init_device_map (char ***map, const char *map_file,
			    int no_floppies);
extern void restore_device_map (char **map);

#ifdef __linux__
extern int is_disk_device (char **map, int drive);
extern int write_to_partition (char **map, int drive, int partition,
			       int offset, int size, const char *buf);
#endif /* __linux__ */
			       
#endif /* DEVICE_MAP_HEADER */
