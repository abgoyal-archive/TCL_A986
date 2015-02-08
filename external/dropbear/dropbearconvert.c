
/* This program converts to/from Dropbear and OpenSSH private-key formats */
#include "includes.h"
#include "signkey.h"
#include "buffer.h"
#include "dbutil.h"
#include "keyimport.h"


static int do_convert(int intype, const char* infile, int outtype,
		const char* outfile);

static void printhelp(char * progname);

static void printhelp(char * progname) {

	fprintf(stderr, "Usage: %s <inputtype> <outputtype> <inputfile> <outputfile>\n\n"
					"CAUTION: This program is for convenience only, and is not secure if used on\n"
					"untrusted input files, ie it could allow arbitrary code execution.\n"
					"All parameters must be specified in order.\n"
					"\n"
					"The input and output types are one of:\n"
					"openssh\n"
					"dropbear\n"
					"\n"
					"Example:\n"
					"dropbearconvert openssh dropbear /etc/ssh/ssh_host_rsa_key /etc/dropbear_rsa_host_key\n",
					progname);
}

#if defined(DBMULTI_dropbearconvert) || !defined(DROPBEAR_MULTI)
#if defined(DBMULTI_dropbearconvert) && defined(DROPBEAR_MULTI)
int dropbearconvert_main(int argc, char ** argv) {
#else 
int main(int argc, char ** argv) {
#endif

	int intype, outtype;
	const char* infile;
	const char* outfile;

#ifdef DEBUG_TRACE
	/* It's hard for it to get in the way _too_ much */
	debug_trace = 1;
#endif

	/* get the commandline options */
	if (argc != 5) {
		fprintf(stderr, "All arguments must be specified\n");
		goto usage;
	}

	/* input type */
	if (argv[1][0] == 'd') {
		intype = KEYFILE_DROPBEAR;
	} else if (argv[1][0] == 'o') {
		intype = KEYFILE_OPENSSH;
	} else {
		fprintf(stderr, "Invalid input key type\n");
		goto usage;
	}

	/* output type */
	if (argv[2][0] == 'd') {
		outtype = KEYFILE_DROPBEAR;
	} else if (argv[2][0] == 'o') {
		outtype = KEYFILE_OPENSSH;
	} else {
		fprintf(stderr, "Invalid output key type\n");
		goto usage;
	}

	/* we don't want output readable by others */
	umask(077);

	infile = argv[3];
	outfile = argv[4];

	return do_convert(intype, infile, outtype, outfile);

usage:
	printhelp(argv[0]);
	return 1;
}
#endif

static int do_convert(int intype, const char* infile, int outtype,
		const char* outfile) {

	sign_key * key = NULL;
	char * keytype = NULL;
	int ret = 1;

	key = import_read(infile, NULL, intype);
	if (!key) {
		fprintf(stderr, "Error reading key from '%s'\n",
				infile);
		goto out;
	}

#ifdef DROPBEAR_RSA
	if (key->rsakey != NULL) {
		keytype = "RSA";
	}
#endif
#ifdef DROPBEAR_DSS
	if (key->dsskey != NULL) {
		keytype = "DSS";
	}
#endif

	fprintf(stderr, "Key is a %s key\n", keytype);

	if (import_write(outfile, key, NULL, outtype) != 1) {
		fprintf(stderr, "Error writing key to '%s'\n", outfile);
	} else {
		fprintf(stderr, "Wrote key to '%s'\n", outfile);
		ret = 0;
	}

out:
	if (key) {
		sign_key_free(key);
	}
	return ret;
}
