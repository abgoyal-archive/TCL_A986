

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/wait.h>

int exitstatus = 0;

typedef void DFL(char *);
DFL *defaultline;

typedef void FILEONLY(char * file);
FILEONLY *internalfunctions;
FILEONLY *externalfunctions;
FILEONLY *symbolsonly;

typedef void FILELINE(char * file, char * line);
FILELINE * singlefunctions;
FILELINE * entity_system;
FILELINE * docsection;

#define MAXLINESZ     2048
#define MAXFILES      250
#define KERNELDOCPATH "scripts/"
#define KERNELDOC     "kernel-doc"
#define DOCBOOK       "-docbook"
#define FUNCTION      "-function"
#define NOFUNCTION    "-nofunction"
#define NODOCSECTIONS "-no-doc-sections"

static char *srctree, *kernsrctree;

static void usage (void)
{
	fprintf(stderr, "Usage: docproc {doc|depend} file\n");
	fprintf(stderr, "Input is read from file.tmpl. Output is sent to stdout\n");
	fprintf(stderr, "doc: frontend when generating kernel documentation\n");
	fprintf(stderr, "depend: generate list of files referenced within file\n");
	fprintf(stderr, "Environment variable SRCTREE: absolute path to sources.\n");
	fprintf(stderr, "                     KBUILD_SRC: absolute path to kernel source tree.\n");
}

static void exec_kernel_doc(char **svec)
{
	pid_t pid;
	int ret;
	char real_filename[PATH_MAX + 1];
	/* Make sure output generated so far are flushed */
	fflush(stdout);
	switch (pid=fork()) {
		case -1:
			perror("fork");
			exit(1);
		case  0:
			memset(real_filename, 0, sizeof(real_filename));
			strncat(real_filename, kernsrctree, PATH_MAX);
			strncat(real_filename, "/" KERNELDOCPATH KERNELDOC,
					PATH_MAX - strlen(real_filename));
			execvp(real_filename, svec);
			fprintf(stderr, "exec ");
			perror(real_filename);
			exit(1);
		default:
			waitpid(pid, &ret ,0);
	}
	if (WIFEXITED(ret))
		exitstatus |= WEXITSTATUS(ret);
	else
		exitstatus = 0xff;
}

/* Types used to create list of all exported symbols in a number of files */
struct symbols
{
	char *name;
};

struct symfile
{
	char *filename;
	struct symbols *symbollist;
	int symbolcnt;
};

struct symfile symfilelist[MAXFILES];
int symfilecnt = 0;

static void add_new_symbol(struct symfile *sym, char * symname)
{
	sym->symbollist =
          realloc(sym->symbollist, (sym->symbolcnt + 1) * sizeof(char *));
	sym->symbollist[sym->symbolcnt++].name = strdup(symname);
}

/* Add a filename to the list */
static struct symfile * add_new_file(char * filename)
{
	symfilelist[symfilecnt++].filename = strdup(filename);
	return &symfilelist[symfilecnt - 1];
}

/* Check if file already are present in the list */
static struct symfile * filename_exist(char * filename)
{
	int i;
	for (i=0; i < symfilecnt; i++)
		if (strcmp(symfilelist[i].filename, filename) == 0)
			return &symfilelist[i];
	return NULL;
}

static void adddep(char * file)		   { printf("\t%s", file); }
static void adddep2(char * file, char * line)     { line = line; adddep(file); }
static void noaction(char * line)		   { line = line; }
static void noaction2(char * file, char * line)   { file = file; line = line; }

/* Echo the line without further action */
static void printline(char * line)               { printf("%s", line); }

static void find_export_symbols(char * filename)
{
	FILE * fp;
	struct symfile *sym;
	char line[MAXLINESZ];
	if (filename_exist(filename) == NULL) {
		char real_filename[PATH_MAX + 1];
		memset(real_filename, 0, sizeof(real_filename));
		strncat(real_filename, srctree, PATH_MAX);
		strncat(real_filename, "/", PATH_MAX - strlen(real_filename));
		strncat(real_filename, filename,
				PATH_MAX - strlen(real_filename));
		sym = add_new_file(filename);
		fp = fopen(real_filename, "r");
		if (fp == NULL)
		{
			fprintf(stderr, "docproc: ");
			perror(real_filename);
			exit(1);
		}
		while (fgets(line, MAXLINESZ, fp)) {
			char *p;
			char *e;
			if (((p = strstr(line, "EXPORT_SYMBOL_GPL")) != NULL) ||
                            ((p = strstr(line, "EXPORT_SYMBOL")) != NULL)) {
				/* Skip EXPORT_SYMBOL{_GPL} */
				while (isalnum(*p) || *p == '_')
					p++;
				/* Remove parentheses & additional whitespace */
				while (isspace(*p))
					p++;
				if (*p != '(')
					continue; /* Syntax error? */
				else
					p++;
				while (isspace(*p))
					p++;
				e = p;
				while (isalnum(*e) || *e == '_')
					e++;
				*e = '\0';
				add_new_symbol(sym, p);
			}
		}
		fclose(fp);
	}
}

static void docfunctions(char * filename, char * type)
{
	int i,j;
	int symcnt = 0;
	int idx = 0;
	char **vec;

	for (i=0; i <= symfilecnt; i++)
		symcnt += symfilelist[i].symbolcnt;
	vec = malloc((2 + 2 * symcnt + 3) * sizeof(char *));
	if (vec == NULL) {
		perror("docproc: ");
		exit(1);
	}
	vec[idx++] = KERNELDOC;
	vec[idx++] = DOCBOOK;
	vec[idx++] = NODOCSECTIONS;
	for (i=0; i < symfilecnt; i++) {
		struct symfile * sym = &symfilelist[i];
		for (j=0; j < sym->symbolcnt; j++) {
			vec[idx++]     = type;
			vec[idx++] = sym->symbollist[j].name;
		}
	}
	vec[idx++]     = filename;
	vec[idx] = NULL;
	printf("<!-- %s -->\n", filename);
	exec_kernel_doc(vec);
	fflush(stdout);
	free(vec);
}
static void intfunc(char * filename) {	docfunctions(filename, NOFUNCTION); }
static void extfunc(char * filename) { docfunctions(filename, FUNCTION);   }

static void singfunc(char * filename, char * line)
{
	char *vec[200]; /* Enough for specific functions */
        int i, idx = 0;
        int startofsym = 1;
	vec[idx++] = KERNELDOC;
	vec[idx++] = DOCBOOK;

        /* Split line up in individual parameters preceded by FUNCTION */
        for (i=0; line[i]; i++) {
                if (isspace(line[i])) {
                        line[i] = '\0';
                        startofsym = 1;
                        continue;
                }
                if (startofsym) {
                        startofsym = 0;
                        vec[idx++] = FUNCTION;
                        vec[idx++] = &line[i];
                }
        }
	vec[idx++] = filename;
	vec[idx] = NULL;
	exec_kernel_doc(vec);
}

static void docsect(char *filename, char *line)
{
	char *vec[6]; /* kerneldoc -docbook -function "section" file NULL */
	char *s;

	for (s = line; *s; s++)
		if (*s == '\n')
			*s = '\0';

	vec[0] = KERNELDOC;
	vec[1] = DOCBOOK;
	vec[2] = FUNCTION;
	vec[3] = line;
	vec[4] = filename;
	vec[5] = NULL;
	exec_kernel_doc(vec);
}

static void parse_file(FILE *infile)
{
	char line[MAXLINESZ];
	char * s;
	while (fgets(line, MAXLINESZ, infile)) {
		if (line[0] == '!') {
			s = line + 2;
			switch (line[1]) {
				case 'E':
					while (*s && !isspace(*s)) s++;
					*s = '\0';
					externalfunctions(line+2);
					break;
				case 'I':
					while (*s && !isspace(*s)) s++;
					*s = '\0';
					internalfunctions(line+2);
					break;
				case 'D':
					while (*s && !isspace(*s)) s++;
                                        *s = '\0';
                                        symbolsonly(line+2);
                                        break;
				case 'F':
					/* filename */
					while (*s && !isspace(*s)) s++;
					*s++ = '\0';
                                        /* function names */
					while (isspace(*s))
						s++;
					singlefunctions(line +2, s);
					break;
				case 'P':
					/* filename */
					while (*s && !isspace(*s)) s++;
					*s++ = '\0';
					/* DOC: section name */
					while (isspace(*s))
						s++;
					docsection(line + 2, s);
					break;
				default:
					defaultline(line);
			}
		}
		else {
			defaultline(line);
		}
	}
	fflush(stdout);
}


int main(int argc, char *argv[])
{
	FILE * infile;

	srctree = getenv("SRCTREE");
	if (!srctree)
		srctree = getcwd(NULL, 0);
	kernsrctree = getenv("KBUILD_SRC");
	if (!kernsrctree || !*kernsrctree)
		kernsrctree = srctree;
	if (argc != 3) {
		usage();
		exit(1);
	}
	/* Open file, exit on error */
	infile = fopen(argv[2], "r");
        if (infile == NULL) {
                fprintf(stderr, "docproc: ");
                perror(argv[2]);
                exit(2);
        }

	if (strcmp("doc", argv[1]) == 0)
	{
		/* Need to do this in two passes.
		 * First pass is used to collect all symbols exported
		 * in the various files;
		 * Second pass generate the documentation.
		 * This is required because some functions are declared
		 * and exported in different files :-((
		 */
		/* Collect symbols */
		defaultline       = noaction;
		internalfunctions = find_export_symbols;
		externalfunctions = find_export_symbols;
		symbolsonly       = find_export_symbols;
		singlefunctions   = noaction2;
		docsection        = noaction2;
		parse_file(infile);

		/* Rewind to start from beginning of file again */
		fseek(infile, 0, SEEK_SET);
		defaultline       = printline;
		internalfunctions = intfunc;
		externalfunctions = extfunc;
		symbolsonly       = printline;
		singlefunctions   = singfunc;
		docsection        = docsect;

		parse_file(infile);
	}
	else if (strcmp("depend", argv[1]) == 0)
	{
		/* Create first part of dependency chain
		 * file.tmpl */
		printf("%s\t", argv[2]);
		defaultline       = noaction;
		internalfunctions = adddep;
		externalfunctions = adddep;
		symbolsonly       = adddep;
		singlefunctions   = adddep2;
		docsection        = adddep2;
		parse_file(infile);
		printf("\n");
	}
	else
	{
		fprintf(stderr, "Unknown option: %s\n", argv[1]);
		exit(1);
	}
	fclose(infile);
	fflush(stdout);
	return exitstatus;
}
