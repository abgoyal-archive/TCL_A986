

#define _FILE_OFFSET_BITS 64

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>

#include "ext2fs/ext2_fs.h"
#include "nls-enable.h"

#undef DEBUG

#ifdef DEBUG
#define WHY(fmt, arg...) { printf("\r%Ld: " fmt, sk, ##arg) ; continue; }
#else
#define WHY(fmt, arg...) { continue; }
#endif

static void usage(void) 
{
	fprintf(stderr, 
		_("Usage:  findsuper device [skipbytes [startkb]]\n"));
	exit(1);
}


int main(int argc, char *argv[])
{
	int skiprate=512;		/* one sector */
	loff_t sk=0, skl=0;
	int fd;
	char *s;
	time_t tm, last = time(0);
	loff_t interval = 1024 * 1024;
	int c, print_jnl_copies = 0;
	const char * device_name;
	struct ext2_super_block ext2;
	/* interesting fields: EXT2_SUPER_MAGIC
	 *      s_blocks_count s_log_block_size s_mtime s_magic s_lastcheck */

#ifdef ENABLE_NLS
	setlocale(LC_MESSAGES, "");
	setlocale(LC_CTYPE, "");
	bindtextdomain(NLS_CAT_NAME, LOCALEDIR);
	textdomain(NLS_CAT_NAME);
#endif

	while ((c = getopt (argc, argv, "j")) != EOF) {
		switch (c) {
		case 'j':
			print_jnl_copies++;
			break;
		default:
			usage();
		}
	}

	if (optind == argc)
		usage();

	device_name = argv[optind++];

	if (optind < argc) {
		skiprate = strtol(argv[optind], &s, 0);
		if (s == argv[optind]) {
			fprintf(stderr,_("skipbytes should be a number, not %s\n"), s);
			exit(1);
		}
		optind++;
	}
	if (skiprate & 0x1ff) {
		fprintf(stderr,
			_("skipbytes must be a multiple of the sector size\n"));
		exit(2);
	}
	if (optind < argc) {
		sk = skl = strtoll(argv[optind], &s, 0) << 10;
		if (s == argv[optind]) {
			fprintf(stderr,
				_("startkb should be a number, not %s\n"), s);
			exit(1);
		}
		optind++;
	}
	if (sk < 0) {
		fprintf(stderr, _("startkb should be positive, not %Lu\n"), sk);
		exit(1);
	}
	
	fd = open(device_name, O_RDONLY);
	if (fd < 0) {
		perror(device_name);
		exit(1);
	}

	/* Now, go looking for the superblock! */
	printf(_("starting at %Lu, with %u byte increments\n"), sk, skiprate);
	if (print_jnl_copies)
		printf(_("[*] probably superblock written in the ext3 "
			 "journal superblock,\n\tso start/end/grp wrong\n"));
	printf(_("byte_offset  byte_start     byte_end  fs_blocks blksz  grp  last_mount_time           sb_uuid label\n"));
	for (; lseek64(fd, sk, SEEK_SET) != -1 &&
	       read(fd, &ext2, 512) == 512; sk += skiprate) {
		static unsigned char last_uuid[16] = "blah";
		unsigned long long bsize, grpsize;
		int jnl_copy, sb_offset;

		if (sk && !(sk & (interval - 1))) {
			time_t now, diff;

			now = time(0);
			diff = now - last;

			if (diff > 0) {
				s = ctime(&now);
				s[24] = 0;
				printf("\r%11Lu: %8LukB/s @ %s", sk,
				       (((sk - skl)) / diff) >> 10, s);
				fflush(stdout);
			}
			if (diff < 5)
				interval <<= 1;
			else if (diff > 20)
				interval >>= 1;
			last = now;
			skl = sk;
		}
		if (ext2.s_magic != EXT2_SUPER_MAGIC)
			continue;
		if (ext2.s_log_block_size > 6)
			WHY("log block size > 6 (%u)\n", ext2.s_log_block_size);
		if (ext2.s_r_blocks_count > ext2.s_blocks_count)
			WHY("r_blocks_count > blocks_count (%u > %u)\n",
			    ext2.s_r_blocks_count, ext2.s_blocks_count);
		if (ext2.s_free_blocks_count > ext2.s_blocks_count)
			WHY("free_blocks_count > blocks_count\n (%u > %u)\n",
			    ext2.s_free_blocks_count, ext2.s_blocks_count);
		if (ext2.s_free_inodes_count > ext2.s_inodes_count)
			WHY("free_inodes_count > inodes_count (%u > %u)\n",
			    ext2.s_free_inodes_count, ext2.s_inodes_count);

		tm = ext2.s_mtime;
		s = ctime(&tm);
		s[24] = 0;
		bsize = 1 << (ext2.s_log_block_size + 10);
		grpsize = bsize * ext2.s_blocks_per_group;
		if (memcmp(ext2.s_uuid, last_uuid, sizeof(last_uuid)) == 0 &&
		    ext2.s_rev_level > 0 && ext2.s_block_group_nr == 0) {
			jnl_copy = 1;
		} else {
			jnl_copy = 0;
			memcpy(last_uuid, ext2.s_uuid, sizeof(last_uuid));
		}
		if (ext2.s_block_group_nr == 0 || bsize == 1024)
			sb_offset = 1024;
		else
			sb_offset = 0;
		if (jnl_copy && !print_jnl_copies)
			continue;
		printf("\r%11Lu %11Lu%s %11Lu%s %9u %5Lu %4u%s %s %02x%02x%02x%02x %s\n",
		       sk, sk - ext2.s_block_group_nr * grpsize - sb_offset,
		       jnl_copy ? "*":" ",
		       sk + ext2.s_blocks_count * bsize -
		            ext2.s_block_group_nr * grpsize - sb_offset,
		       jnl_copy ? "*" : " ", ext2.s_blocks_count, bsize,
		       ext2.s_block_group_nr, jnl_copy ? "*" : " ", s,
		       ext2.s_uuid[0], ext2.s_uuid[1],
		       ext2.s_uuid[2], ext2.s_uuid[3], ext2.s_volume_name);
	}
	printf(_("\n%11Lu: finished with errno %d\n"), sk, errno);
	close(fd);

	return errno;
}
