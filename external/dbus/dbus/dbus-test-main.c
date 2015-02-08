
/* -*- mode: C; c-file-style: "gnu" -*- */


#include "dbus-types.h"
#include "dbus-test.h"
#include <stdio.h>
#include <stdlib.h>
#include <locale.h>

int
main (int    argc,
      char **argv)
{
  const char *test_data_dir;
  const char *specific_test;

  setlocale(LC_ALL, "");

  
  if (argc > 1)
    test_data_dir = argv[1];
  else
    test_data_dir = NULL;

  if (argc > 2)
    specific_test = argv[2];
  else
    specific_test = NULL;
  
  dbus_internal_do_not_use_run_tests (test_data_dir, specific_test);
  
  return 0;
}
