

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <ctype.h>
#include <arpa/inet.h>

#define INT_CONF ntohl(0x434f4e46)
#define INT_ONFI ntohl(0x4f4e4649)
#define INT_NFIG ntohl(0x4e464947)
#define INT_FIG_ ntohl(0x4649475f)

char *target;
char *depfile;
char *cmdline;

static void usage(void)
{
	fprintf(stderr, "Usage: fixdep <depfile> <target> <cmdline>\n");
	exit(1);
}

static void print_cmdline(void)
{
	printf("cmd_%s := %s\n\n", target, cmdline);
}

char * str_config  = NULL;
int    size_config = 0;
int    len_config  = 0;

static void grow_config(int len)
{
	while (len_config + len > size_config) {
		if (size_config == 0)
			size_config = 2048;
		str_config = realloc(str_config, size_config *= 2);
		if (str_config == NULL)
			{ perror("fixdep:malloc"); exit(1); }
	}
}



static int is_defined_config(const char * name, int len)
{
	const char * pconfig;
	const char * plast = str_config + len_config - len;
	for ( pconfig = str_config + 1; pconfig < plast; pconfig++ ) {
		if (pconfig[ -1] == '\n'
		&&  pconfig[len] == '\n'
		&&  !memcmp(pconfig, name, len))
			return 1;
	}
	return 0;
}

static void define_config(const char * name, int len)
{
	grow_config(len + 1);

	memcpy(str_config+len_config, name, len);
	len_config += len;
	str_config[len_config++] = '\n';
}

static void clear_config(void)
{
	len_config = 0;
	define_config("", 0);
}

static void use_config(char *m, int slen)
{
	char s[PATH_MAX];
	char *p;

	if (is_defined_config(m, slen))
	    return;

	define_config(m, slen);

	memcpy(s, m, slen); s[slen] = 0;

	for (p = s; p < s + slen; p++) {
		if (*p == '_')
			*p = '/';
		else
			*p = tolower((int)*p);
	}
	printf("    $(wildcard include/config/%s.h) \\\n", s);
}

static void parse_config_file(char *map, size_t len)
{
	int *end = (int *) (map + len);
	/* start at +1, so that p can never be < map */
	int *m   = (int *) map + 1;
	char *p, *q;

	for (; m < end; m++) {
		if (*m == INT_CONF) { p = (char *) m  ; goto conf; }
		if (*m == INT_ONFI) { p = (char *) m-1; goto conf; }
		if (*m == INT_NFIG) { p = (char *) m-2; goto conf; }
		if (*m == INT_FIG_) { p = (char *) m-3; goto conf; }
		continue;
	conf:
		if (p > map + len - 7)
			continue;
		if (memcmp(p, "CONFIG_", 7))
			continue;
		for (q = p + 7; q < map + len; q++) {
			if (!(isalnum(*q) || *q == '_'))
				goto found;
		}
		continue;

	found:
		if (!memcmp(q - 7, "_MODULE", 7))
			q -= 7;
		if( (q-p-7) < 0 )
			continue;
		use_config(p+7, q-p-7);
	}
}

/* test is s ends in sub */
static int strrcmp(char *s, char *sub)
{
	int slen = strlen(s);
	int sublen = strlen(sub);

	if (sublen > slen)
		return 1;

	return memcmp(s + slen - sublen, sub, sublen);
}

static void do_config_file(char *filename)
{
	struct stat st;
	int fd;
	void *map;

	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "fixdep: ");
		perror(filename);
		exit(2);
	}
	fstat(fd, &st);
	if (st.st_size == 0) {
		close(fd);
		return;
	}
	map = mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	if ((long) map == -1) {
		perror("fixdep: mmap");
		close(fd);
		return;
	}

	parse_config_file(map, st.st_size);

	munmap(map, st.st_size);

	close(fd);
}

static void parse_dep_file(void *map, size_t len)
{
	char *m = map;
	char *end = m + len;
	char *p;
	char s[PATH_MAX];

	p = strchr(m, ':');
	if (!p) {
		fprintf(stderr, "fixdep: parse error\n");
		exit(1);
	}
	memcpy(s, m, p-m); s[p-m] = 0;
	printf("deps_%s := \\\n", target);
	m = p+1;

	clear_config();

	while (m < end) {
		while (m < end && (*m == ' ' || *m == '\\' || *m == '\n'))
			m++;
		p = m;
		while (p < end && *p != ' ') p++;
		if (p == end) {
			do p--; while (!isalnum(*p));
			p++;
		}
		memcpy(s, m, p-m); s[p-m] = 0;
		if (strrcmp(s, "include/generated/autoconf.h") &&
		    strrcmp(s, "arch/um/include/uml-config.h") &&
		    strrcmp(s, ".ver")) {
			printf("  %s \\\n", s);
			do_config_file(s);
		}
		m = p + 1;
	}
	printf("\n%s: $(deps_%s)\n\n", target, target);
	printf("$(deps_%s):\n", target);
}

static void print_deps(void)
{
	struct stat st;
	int fd;
	void *map;

	fd = open(depfile, O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "fixdep: ");
		perror(depfile);
		exit(2);
	}
	fstat(fd, &st);
	if (st.st_size == 0) {
		fprintf(stderr,"fixdep: %s is empty\n",depfile);
		close(fd);
		return;
	}
	map = mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	if ((long) map == -1) {
		perror("fixdep: mmap");
		close(fd);
		return;
	}

	parse_dep_file(map, st.st_size);

	munmap(map, st.st_size);

	close(fd);
}

static void traps(void)
{
	static char test[] __attribute__((aligned(sizeof(int)))) = "CONF";
	int *p = (int *)test;

	if (*p != INT_CONF) {
		fprintf(stderr, "fixdep: sizeof(int) != 4 or wrong endianess? %#x\n",
			*p);
		exit(2);
	}
}

int main(int argc, char *argv[])
{
	traps();

	if (argc != 4)
		usage();

	depfile = argv[1];
	target = argv[2];
	cmdline = argv[3];

	print_cmdline();
	print_deps();

	return 0;
}
