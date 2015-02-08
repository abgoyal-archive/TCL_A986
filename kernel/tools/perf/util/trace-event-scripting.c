

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include "../perf.h"
#include "util.h"
#include "trace-event.h"

struct scripting_context *scripting_context;

static int stop_script_unsupported(void)
{
	return 0;
}

static void process_event_unsupported(int cpu __unused,
				      void *data __unused,
				      int size __unused,
				      unsigned long long nsecs __unused,
				      char *comm __unused)
{
}

static void print_python_unsupported_msg(void)
{
	fprintf(stderr, "Python scripting not supported."
		"  Install libpython and rebuild perf to enable it.\n"
		"For example:\n  # apt-get install python-dev (ubuntu)"
		"\n  # yum install python-devel (Fedora)"
		"\n  etc.\n");
}

static int python_start_script_unsupported(const char *script __unused,
					   int argc __unused,
					   const char **argv __unused)
{
	print_python_unsupported_msg();

	return -1;
}

static int python_generate_script_unsupported(const char *outfile __unused)
{
	print_python_unsupported_msg();

	return -1;
}

struct scripting_ops python_scripting_unsupported_ops = {
	.name = "Python",
	.start_script = python_start_script_unsupported,
	.stop_script = stop_script_unsupported,
	.process_event = process_event_unsupported,
	.generate_script = python_generate_script_unsupported,
};

static void register_python_scripting(struct scripting_ops *scripting_ops)
{
	int err;
	err = script_spec_register("Python", scripting_ops);
	if (err)
		die("error registering Python script extension");

	err = script_spec_register("py", scripting_ops);
	if (err)
		die("error registering py script extension");

	scripting_context = malloc(sizeof(struct scripting_context));
}

#ifdef NO_LIBPYTHON
void setup_python_scripting(void)
{
	register_python_scripting(&python_scripting_unsupported_ops);
}
#else
struct scripting_ops python_scripting_ops;

void setup_python_scripting(void)
{
	register_python_scripting(&python_scripting_ops);
}
#endif

static void print_perl_unsupported_msg(void)
{
	fprintf(stderr, "Perl scripting not supported."
		"  Install libperl and rebuild perf to enable it.\n"
		"For example:\n  # apt-get install libperl-dev (ubuntu)"
		"\n  # yum install 'perl(ExtUtils::Embed)' (Fedora)"
		"\n  etc.\n");
}

static int perl_start_script_unsupported(const char *script __unused,
					 int argc __unused,
					 const char **argv __unused)
{
	print_perl_unsupported_msg();

	return -1;
}

static int perl_generate_script_unsupported(const char *outfile __unused)
{
	print_perl_unsupported_msg();

	return -1;
}

struct scripting_ops perl_scripting_unsupported_ops = {
	.name = "Perl",
	.start_script = perl_start_script_unsupported,
	.stop_script = stop_script_unsupported,
	.process_event = process_event_unsupported,
	.generate_script = perl_generate_script_unsupported,
};

static void register_perl_scripting(struct scripting_ops *scripting_ops)
{
	int err;
	err = script_spec_register("Perl", scripting_ops);
	if (err)
		die("error registering Perl script extension");

	err = script_spec_register("pl", scripting_ops);
	if (err)
		die("error registering pl script extension");

	scripting_context = malloc(sizeof(struct scripting_context));
}

#ifdef NO_LIBPERL
void setup_perl_scripting(void)
{
	register_perl_scripting(&perl_scripting_unsupported_ops);
}
#else
struct scripting_ops perl_scripting_ops;

void setup_perl_scripting(void)
{
	register_perl_scripting(&perl_scripting_ops);
}
#endif
