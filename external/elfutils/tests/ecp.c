
#include <errno.h>
#include <error.h>
#include <fcntl.h>
#include <gelf.h>
#include <stdlib.h>
#include <unistd.h>

int
main (int argc, char *argv[])
{
  if (argc < 3)
    error (EXIT_FAILURE, 0, "usage: %s FROMNAME TONAME", argv[0]);

  elf_version (EV_CURRENT);

  int infd = open (argv[1], O_RDONLY);
  if (infd == -1)
    error (EXIT_FAILURE, errno, "cannot open input file '%s'", argv[1]);

  Elf *inelf = elf_begin (infd, ELF_C_READ, NULL);
  if (inelf == NULL)
    error (EXIT_FAILURE, 0, "problems opening '%s' as ELF file: %s",
	   argv[1], elf_errmsg (-1));

  int outfd = creat (argv[2], 0666);
  if (outfd == -1)
    error (EXIT_FAILURE, errno, "cannot open output file '%s'", argv[2]);

  Elf *outelf = elf_begin (outfd, ELF_C_WRITE, NULL);
  if (outelf == NULL)
    error (EXIT_FAILURE, 0, "problems opening '%s' as ELF file: %s",
	   argv[2], elf_errmsg (-1));

  gelf_newehdr (outelf, gelf_getclass (inelf));

  GElf_Ehdr ehdr_mem;
  GElf_Ehdr *ehdr;
  gelf_update_ehdr (outelf, (ehdr = gelf_getehdr (inelf, &ehdr_mem)));

  if (ehdr->e_phnum > 0)
    {
      int cnt;

      if (gelf_newphdr (outelf, ehdr->e_phnum) == 0)
	error (EXIT_FAILURE, 0, "cannot create program header: %s",
	       elf_errmsg (-1));

      for (cnt = 0; cnt < ehdr->e_phnum; ++cnt)
	{
	  GElf_Phdr phdr_mem;

	  gelf_update_phdr (outelf, cnt, gelf_getphdr (inelf, cnt, &phdr_mem));
	}
    }

  Elf_Scn *scn = NULL;
  while ((scn = elf_nextscn (inelf, scn)) != NULL)
    {
      Elf_Scn *newscn = elf_newscn (outelf);

      GElf_Shdr shdr_mem;
      gelf_update_shdr (newscn, gelf_getshdr (scn, &shdr_mem));

      *elf_newdata (newscn) = *elf_getdata (scn, NULL);
    }

  elf_flagelf (outelf, ELF_C_SET, ELF_F_LAYOUT);

  if (elf_update (outelf, ELF_C_WRITE) == -1)
    error (EXIT_FAILURE, 0, "elf_update failed: %s", elf_errmsg (-1));

  close (outfd);

  return 0;
}
