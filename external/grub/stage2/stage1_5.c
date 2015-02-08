
#include "shared.h"

static int saved_sector = -1;

static void
disk_read_savesect_func (int sector, int offset, int length)
{
  saved_sector = sector;
}

void
cmain (void)
{
  grub_printf ("\n\nGRUB loading, please wait...\n");

  /*
   *  Here load the true second-stage boot-loader.
   */

  if (grub_open (config_file))
    {
      int ret;

      disk_read_hook = disk_read_savesect_func;
      grub_read ((char *) 0x8000, SECTOR_SIZE * 2);
      disk_read_hook = NULL;

      /* Sanity check: catch an internal error.  */
      if (saved_sector == -1)
	{
	  grub_printf ("internal error: the second sector of Stage 2 is unknown.");
	  stop ();
	}
      
      ret = grub_read ((char *) 0x8000 + SECTOR_SIZE * 2, -1);
      
      grub_close ();

      if (ret)
	chain_stage2 (0, 0x8200, saved_sector);
    }

  /*
   *  If not, then print error message and die.
   */

  print_error ();

  stop ();
}
