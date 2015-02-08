/* cmdline.c - the device-independent GRUB text command line */

#include <shared.h>

#ifdef SUPPORT_DISKLESS
# define GRUB	1
# include <etherboot.h>
#endif

grub_jmp_buf restart_cmdline_env;

char *
skip_to (int after_equal, char *cmdline)
{
  /* Skip until we hit whitespace, or maybe an equal sign. */
  while (*cmdline && *cmdline != ' ' && *cmdline != '\t' &&
	 ! (after_equal && *cmdline == '='))
    cmdline ++;

  /* Skip whitespace, and maybe equal signs. */
  while (*cmdline == ' ' || *cmdline == '\t' ||
	 (after_equal && *cmdline == '='))
    cmdline ++;

  return cmdline;
}

/* Print a helpful message for the command-line interface.  */
void
print_cmdline_message (int forever)
{
  printf (" [ Minimal BASH-like line editing is supported.  For the first word, TAB\n"
	  "   lists possible command completions.  Anywhere else TAB lists the possible\n"
	  "   completions of a device/filename.%s ]\n",
	  (forever ? "" : "  ESC at any time exits."));
}

struct builtin *
find_command (char *command)
{
  char *ptr;
  char c;
  struct builtin **builtin;

  /* Find the first space and terminate the command name.  */
  ptr = command;
  while (*ptr && *ptr != ' ' && *ptr != '\t' && *ptr != '=')
    ptr ++;

  c = *ptr;
  *ptr = 0;

  /* Seek out the builtin whose command name is COMMAND.  */
  for (builtin = builtin_table; *builtin != 0; builtin++)
    {
      int ret = grub_strcmp (command, (*builtin)->name);

      if (ret == 0)
	{
	  /* Find the builtin for COMMAND.  */
	  *ptr = c;
	  return *builtin;
	}
      else if (ret < 0)
	break;
    }

  /* Cannot find COMMAND.  */
  errnum = ERR_UNRECOGNIZED;
  *ptr = c;
  return 0;
}

/* Initialize the data for the command-line.  */
static void
init_cmdline (void)
{
  /* Initialization.  */
  saved_drive = boot_drive;
  saved_partition = install_partition;
  current_drive = GRUB_INVALID_DRIVE;
  errnum = 0;
  count_lines = -1;
  
  /* Restore memory probe state.  */
  mbi.mem_upper = saved_mem_upper;
  if (mbi.mmap_length)
    mbi.flags |= MB_INFO_MEM_MAP;

  /* Initialize the data for the builtin commands.  */
  init_builtins ();
}

void
enter_cmdline (char *heap, int forever)
{
  /* Initialize the data and print a message.  */
  init_cmdline ();
  grub_setjmp (restart_cmdline_env);
  init_page ();
#ifdef SUPPORT_DISKLESS
  print_network_configuration ();
  grub_putchar ('\n');
#endif
  print_cmdline_message (forever);
  
  while (1)
    {
      struct builtin *builtin;
      char *arg;

      *heap = 0;
      print_error ();
      errnum = ERR_NONE;

      /* Get the command-line with the minimal BASH-like interface.  */
      if (get_cmdline (PACKAGE "> ", heap, 2048, 0, 1))
	return;

      /* If there was no command, grab a new one. */
      if (! heap[0])
	continue;

      /* Find a builtin.  */
      builtin = find_command (heap);
      if (! builtin)
	continue;

      /* If BUILTIN cannot be run in the command-line, skip it.  */
      if (! (builtin->flags & BUILTIN_CMDLINE))
	{
	  errnum = ERR_UNRECOGNIZED;
	  continue;
	}

      /* Invalidate the cache, because the user may exchange removable
	 disks.  */
      buf_drive = -1;

      /* Start to count lines, only if the internal pager is in use.  */
      if (use_pager)
	count_lines = 0;
      
      /* Run BUILTIN->FUNC.  */
      arg = skip_to (1, heap);
      (builtin->func) (arg, BUILTIN_CMDLINE);

      /* Finish the line count.  */
      count_lines = -1;
    }
}

int
run_script (char *script, char *heap)
{
  char *old_entry;
  char *cur_entry = script;

  /* Initialize the data.  */
  init_cmdline ();

  while (1)
    {
      struct builtin *builtin;
      char *arg;

      print_error ();

      if (errnum)
	{
	  errnum = ERR_NONE;

	  /* If a fallback entry is defined, don't prompt a user's
	     intervention.  */
	  if (fallback_entryno < 0)
	    {
	      grub_printf ("\nPress any key to continue...");
	      (void) getkey ();
	    }
	  
	  return 1;
	}

      /* Copy the first string in CUR_ENTRY to HEAP.  */
      old_entry = cur_entry;
      while (*cur_entry++)
	;

      grub_memmove (heap, old_entry, (int) cur_entry - (int) old_entry);
      if (! *heap)
	{
	  /* If there is no more command in SCRIPT...  */

	  /* If any kernel is not loaded, just exit successfully.  */
	  if (kernel_type == KERNEL_TYPE_NONE)
	    return 0;

	  /* Otherwise, the command boot is run implicitly.  */
	  grub_memmove (heap, "boot", 5);
	}

      /* Find a builtin.  */
      builtin = find_command (heap);
      if (! builtin)
	{
	  grub_printf ("%s\n", old_entry);
	  continue;
	}

      if (! (builtin->flags & BUILTIN_NO_ECHO))
	grub_printf ("%s\n", old_entry);

      /* If BUILTIN cannot be run in the command-line, skip it.  */
      if (! (builtin->flags & BUILTIN_CMDLINE))
	{
	  errnum = ERR_UNRECOGNIZED;
	  continue;
	}

      /* Invalidate the cache, because the user may exchange removable
	 disks.  */
      buf_drive = -1;

      /* Run BUILTIN->FUNC.  */
      arg = skip_to (1, heap);
      (builtin->func) (arg, BUILTIN_SCRIPT);
    }
}
