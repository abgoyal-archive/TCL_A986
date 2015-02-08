

#ifndef EXT_H
#define EXT_H

#include <sys/types.h>

#include "dosfs.h"

#define	LOSTDIR	"LOST.DIR"

extern int alwaysno;	/* assume "no" for all questions */
extern int alwaysyes;	/* assume "yes" for all questions */
extern int preen;	/* we are preening */
extern int rdonly;	/* device is opened read only (supersedes above) */
extern int skipclean;	/* skip clean file systems if preening */
extern int debugmessage;	/* print verbose message for debugging */

extern struct dosDirEntry *rootDir;

int ask(int, const char *, ...);

int checkdirty(int, struct bootblock *);

int checkfilesys(const char *);

#define	FSOK		0		/* Check was OK */
#define	FSBOOTMOD	1		/* Boot block was modified */
#define	FSDIRMOD	2		/* Some directory was modified */
#define	FSFATMOD	4		/* The FAT was modified */
#define	FSERROR		8		/* Some unrecovered error remains */
#define	FSFATAL		16		/* Some unrecoverable error occured */
#define FSDIRTY		32		/* File system is dirty */
#define FSFIXFAT	64		/* Fix file system FAT */

int readboot(int, struct bootblock *);

int writefsinfo(int, struct bootblock *);

int readfat(int, struct bootblock *, int, struct fatEntry **);

int comparefat(struct bootblock *, struct fatEntry *, struct fatEntry *, int);

int checkfat(struct bootblock *, struct fatEntry *);

int writefat(int, struct bootblock *, struct fatEntry *, int);

int resetDosDirSection(struct bootblock *, struct fatEntry *);
void finishDosDirSection(void);
int handleDirTree(int, struct bootblock *, struct fatEntry *);

int checklost(int, struct bootblock *, struct fatEntry *);
int reconnect(int, struct bootblock *, struct fatEntry *, cl_t);
void finishlf(void);

char *rsrvdcltype(cl_t);

void clearchain(struct bootblock *, struct fatEntry *, cl_t);

#endif
