
#ifndef _ELF_KNOWLEDGE_H
#define _ELF_KNOWLEDGE_H	1


/* Test whether a section can be stripped or not.  */
#define SECTION_STRIP_P(ebl, elf, ehdr, shdr, name, remove_comment, remove_debug) \
  (/* Sections which are allocated are not removed.  */			      \
   ((shdr)->sh_flags & SHF_ALLOC) == 0					      \
   /* We never remove .note sections.  */				      \
   && (shdr)->sh_type != SHT_NOTE					      \
   /* If only debug information should be removed check the name.  There      \
      is unfortunately no way.  */					      \
   && (!remove_debug							      \
       || ebl_debugscn_p (ebl, name)					      \
       || (((shdr)->sh_type == SHT_RELA || (shdr)->sh_type == SHT_REL)	      \
	   && ({ Elf_Scn *scn_l = elf_getscn (elf, (shdr)->sh_info);	      \
		 GElf_Shdr shdr_mem_l;					      \
		 GElf_Shdr *shdr_l = gelf_getshdr (scn_l, &shdr_mem_l);	      \
		 const char *s_l;					      \
		 shdr_l != NULL						      \
		   && (s_l = elf_strptr (elf,				      \
					 ((GElf_Ehdr *) (ehdr))->e_shstrndx,  \
					 shdr_l->sh_name)) != NULL	      \
		   && ebl_debugscn_p (ebl, s_l); })))			      \
   && ((shdr)->sh_type != SHT_PROGBITS					      \
   /* Never remove .gnu.warning.* sections.  */				      \
       || (strncmp (name, ".gnu.warning.", sizeof ".gnu.warning." - 1) != 0   \
   /* We remove .comment sections only if explicitly told to do so.  */	      \
	   && ((remove_comment) || strcmp (name, ".comment") != 0)))	      \
   /* So far we do not remove any of the non-standard sections.		      \
      XXX Maybe in future.  */						      \
   && (shdr)->sh_type < SHT_NUM)


#define SH_INFO_LINK_P(Shdr) \
  ((Shdr)->sh_type == SHT_REL || (Shdr)->sh_type == SHT_RELA		      \
   || ((Shdr)->sh_flags & SHF_INFO_LINK) != 0)


#define SH_FLAGS_COMBINE(Flags1, Flags2) \
  (((Flags1 | Flags2)							      \
    & (SHF_WRITE | SHF_ALLOC | SHF_EXECINSTR | SHF_LINK_ORDER		      \
       | SHF_OS_NONCONFORMING | SHF_GROUP))				      \
   | (Flags1 & Flags2 & (SHF_MERGE | SHF_STRINGS | SHF_INFO_LINK)))

#define SH_FLAGS_IMPORTANT(Flags) \
  ((Flags) & ~((GElf_Xword) 0 | SHF_LINK_ORDER | SHF_OS_NONCONFORMING))


#define SH_ENTSIZE_HASH(Ehdr) \
  ((Ehdr)->e_machine == EM_ALPHA					      \
   || ((Ehdr)->e_machine == EM_S390					      \
       && (Ehdr)->e_ident[EI_CLASS] == ELFCLASS64) ? 8 : 4)

#endif	/* elf-knowledge.h */
