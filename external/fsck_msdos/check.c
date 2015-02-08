


#include <sys/cdefs.h>
#ifndef lint
__RCSID("$NetBSD: check.c,v 1.10 2000/04/25 23:02:51 jdolecek Exp $");
static const char rcsid[] =
  "$FreeBSD: src/sbin/fsck_msdosfs/check.c,v 1.10 2004/02/05 15:47:46 bde Exp $";
#endif /* not lint */

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include "ext.h"
#include "fsutil.h"

#define FAT_COMPARE_MAX_KB 4096

int
checkfilesys(const char *fname)
{
	int dosfs;
	struct bootblock boot;
	struct fatEntry *fat = NULL;
	int i;
	int mod = 0;
        int quiet = 0;
        int skip_fat_compare = 0;

	rdonly = alwaysno;
	if (!quiet)
		printf("** %s", fname);

	dosfs = open(fname, rdonly ? O_RDONLY : O_RDWR, 0);
	if (dosfs < 0 && !rdonly) {
		dosfs = open(fname, O_RDONLY, 0);
		if (dosfs >= 0)
			pwarn(" (NO WRITE)\n");
		else if (!quiet)
			printf("\n");
		rdonly = 1;
	} else if (!quiet)
		printf("\n");

	if (dosfs < 0) {
		perror("Can't open");
		return 8;
	}

	if (readboot(dosfs, &boot) == FSFATAL) {
		close(dosfs);
		printf("\n");
		return 8;
	}

	if (skipclean && preen && checkdirty(dosfs, &boot)) {
		printf("%s: ", fname);
		printf("FILESYSTEM CLEAN; SKIPPING CHECKS\n");
		close(dosfs);
		return 0;
	}

        if (((boot.FATsecs * boot.BytesPerSec) / 1024) > FAT_COMPARE_MAX_KB)
            skip_fat_compare = 1;

	if (!quiet)  {
                if (skip_fat_compare) 
                        printf("** Phase 1 - Read FAT (compare skipped)\n");
		else if (boot.ValidFat < 0)
			printf("** Phase 1 - Read and Compare FATs\n");
		else
			printf("** Phase 1 - Read FAT\n");
	}

	mod |= readfat(dosfs, &boot, boot.ValidFat >= 0 ? boot.ValidFat : 0, &fat);
	if (mod & FSERROR) {
		printf("FSERROR after readfat()\n");
	}
	if (mod & FSFATAL) {
		printf("\nFatal error during readfat()\n");
		close(dosfs);
		return 8;
	}

	if (!skip_fat_compare && boot.ValidFat < 0)
		for (i = 1; i < (int)boot.FATs; i++) {
			struct fatEntry *currentFat;

			mod |= readfat(dosfs, &boot, i, &currentFat);

			if (mod & FSFATAL) {
				printf("Fatal error during readfat() for comparison\n");
				free(fat);
				(void)close(dosfs);
				return 8;
			}

			mod |= comparefat(&boot, fat, currentFat, i);
			free(currentFat);
			if (mod & FSFATAL) {
				printf("Fatal error during FAT comparison\n");
				free(fat);
				(void)close(dosfs);
				return 8;
			}
		}

	if (!quiet)
		printf("** Phase 2 - Check Cluster Chains\n");

	mod |= checkfat(&boot, fat);
	if (mod & FSERROR) {
		printf("FSERROR after checkfat()\n");
	}
	if (mod & FSFATAL) {
		printf("Fatal error during FAT check\n");
		free(fat);
		(void)close(dosfs);
		return 8;
	}
	/* delay writing FATs */

	if (!quiet)
		printf("** Phase 3 - Checking Directories\n");

	mod |= resetDosDirSection(&boot, fat);
	if (mod & FSERROR) {
		printf("FSERROR after resetDosDirSection()\n");
	}
	if (mod & FSFATAL) {
		printf("Fatal error during resetDosDirSection()\n");
		free(fat);
		close(dosfs);
		return 8;
	}
	/* delay writing FATs */

	mod |= handleDirTree(dosfs, &boot, fat);
	if (mod & FSERROR) {
		printf("FSERROR after handleDirTree()\n");
		/* try to recovery again */
		mod &= ~FSERROR;
		mod |= handleDirTree(dosfs, &boot, fat);
	}
	if (mod & FSFATAL) {
		printf("Fatal error during handleDirTree()\n");
		finishDosDirSection();
		free(fat);
		close(dosfs);
		return 8;
	}

	if (!quiet)
		printf("** Phase 4 - Checking for Lost Files\n");

	mod |= checklost(dosfs, &boot, fat);
	if (mod & FSERROR) {
		printf("FSERROR after checklost()\n");
	}
	if (mod & FSFATAL) {
		printf("Fatal error during checklost()\n");
		finishDosDirSection();
		free(fat);
		(void)close(dosfs);
		return 8;
	}

	/* now write the FATs */
	if (mod & FSFATMOD) {
		if (ask(1, "Update FATs")) {
			mod |= writefat(dosfs, &boot, fat, mod & FSFIXFAT);
			if (mod & FSFATAL) {
				printf("Fatal error during writefat()\n");
				finishDosDirSection();
				free(fat);
				(void)close(dosfs);
				return 8;
			}
		} else
			mod |= FSERROR;
	}

	if (boot.NumBad)
		pwarn("%d files, %d free (%d clusters), %d bad (%d clusters)\n",
		      boot.NumFiles,
		      boot.NumFree * boot.ClusterSize / 1024, boot.NumFree,
		      boot.NumBad * boot.ClusterSize / 1024, boot.NumBad);
	else
		pwarn("%d files, %d free (%d clusters)\n",
		      boot.NumFiles,
		      boot.NumFree * boot.ClusterSize / 1024, boot.NumFree);

	if (mod && (mod & FSERROR) == 0) {
		if (mod & FSDIRTY) {
			if (ask(1, "MARK FILE SYSTEM CLEAN") == 0)
				mod &= ~FSDIRTY;

			if (mod & FSDIRTY) {
				pwarn("MARKING FILE SYSTEM CLEAN\n");
				mod |= writefat(dosfs, &boot, fat, 1);
			} else {
				pwarn("\n***** FILE SYSTEM IS LEFT MARKED AS DIRTY *****\n");
				mod |= FSERROR; /* file system not clean */
			}
		}
	}

	finishDosDirSection();
	free(fat);
	(void)close(dosfs);

	if (mod & FSFATAL) {
		pwarn("\n***** FSFATAL *****\n");
		return 8;
	}
	if (mod & FSERROR) {
		pwarn("\n***** FSERROR *****\n");
		return 8;
	}
	if (mod) {
		pwarn("\n***** FILE SYSTEM WAS MODIFIED *****\n");
		return 4; 
	}

	return 0;
}
