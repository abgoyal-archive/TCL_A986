
#ifndef _LIBELF_H
#define _LIBELF_H 1

#include <sys/types.h>

/* Get the ELF types.  */
#include <elf.h>


/* Known translation types.  */
typedef enum
{
  ELF_T_BYTE,                   /* unsigned char */
  ELF_T_ADDR,                   /* Elf32_Addr, Elf64_Addr, ... */
  ELF_T_DYN,                    /* Dynamic section record.  */
  ELF_T_EHDR,                   /* ELF header.  */
  ELF_T_HALF,                   /* Elf32_Half, Elf64_Half, ... */
  ELF_T_OFF,                    /* Elf32_Off, Elf64_Off, ... */
  ELF_T_PHDR,                   /* Program header.  */
  ELF_T_RELA,                   /* Relocation entry with addend.  */
  ELF_T_REL,                    /* Relocation entry.  */
  ELF_T_SHDR,                   /* Section header.  */
  ELF_T_SWORD,                  /* Elf32_Sword, Elf64_Sword, ... */
  ELF_T_SYM,                    /* Symbol record.  */
  ELF_T_WORD,                   /* Elf32_Word, Elf64_Word, ... */
  ELF_T_XWORD,                  /* Elf32_Xword, Elf64_Xword, ... */
  ELF_T_SXWORD,                 /* Elf32_Sxword, Elf64_Sxword, ... */
  ELF_T_VDEF,                   /* Elf32_Verdef, Elf64_Verdef, ... */
  ELF_T_VDAUX,                  /* Elf32_Verdaux, Elf64_Verdaux, ... */
  ELF_T_VNEED,                  /* Elf32_Verneed, Elf64_Verneed, ... */
  ELF_T_VNAUX,                  /* Elf32_Vernaux, Elf64_Vernaux, ... */
  ELF_T_NHDR,                   /* Elf32_Nhdr, Elf64_Nhdr, ... */
  ELF_T_SYMINFO,		/* Elf32_Syminfo, Elf64_Syminfo, ... */
  ELF_T_MOVE,			/* Elf32_Move, Elf64_Move, ... */
  ELF_T_LIB,			/* Elf32_Lib, Elf64_Lib, ... */
  /* Keep this the last entry.  */
  ELF_T_NUM
} Elf_Type;

/* Descriptor for data to be converted to or from memory format.  */
typedef struct
{
  void *d_buf;			/* Pointer to the actual data.  */
  Elf_Type d_type;		/* Type of this piece of data.  */
  unsigned int d_version;	/* ELF version.  */
  size_t d_size;		/* Size in bytes.  */
  off_t d_off;			/* Offset into section.  */
  size_t d_align;		/* Alignment in section.  */
} Elf_Data;


/* Commands for `...'.  */
typedef enum
{
  ELF_C_NULL,			/* Nothing, terminate, or compute only.  */
  ELF_C_READ,			/* Read .. */
  ELF_C_RDWR,			/* Read and write .. */
  ELF_C_WRITE,			/* Write .. */
  ELF_C_CLR,			/* Clear flag.  */
  ELF_C_SET,			/* Set flag.  */
  ELF_C_FDDONE,			/* Signal that file descriptor will not be
				   used anymore.  */
  ELF_C_FDREAD,			/* Read rest of data so that file descriptor
				   is not used anymore.  */
  /* The following are extensions.  */
  ELF_C_READ_MMAP,		/* Read, but mmap the file if possible.  */
  ELF_C_RDWR_MMAP,		/* Read and write, with mmap.  */
  ELF_C_WRITE_MMAP,		/* Write, with mmap.  */
  ELF_C_READ_MMAP_PRIVATE,	/* Read, but memory is writable, results are
				   not written to the file.  */
  ELF_C_EMPTY,			/* Copy basic file data but not the content. */
  /* Keep this the last entry.  */
  ELF_C_NUM
} Elf_Cmd;


/* Flags for the ELF structures.  */
enum
{
  ELF_F_DIRTY = 0x1,
#define ELF_F_DIRTY		ELF_F_DIRTY
  ELF_F_LAYOUT = 0x4,
#define ELF_F_LAYOUT		ELF_F_LAYOUT
  ELF_F_PERMISSIVE = 0x8
#define ELF_F_PERMISSIVE	ELF_F_PERMISSIVE
};


/* Identification values for recognized object files.  */
typedef enum
{
  ELF_K_NONE,			/* Unknown.  */
  ELF_K_AR,			/* Archive.  */
  ELF_K_COFF,			/* Stupid old COFF.  */
  ELF_K_ELF,			/* ELF file.  */
  /* Keep this the last entry.  */
  ELF_K_NUM
} Elf_Kind;


/* Archive member header.  */
typedef struct
{
  char *ar_name;		/* Name of archive member.  */
  time_t ar_date;		/* File date.  */
  uid_t ar_uid;			/* User ID.  */
  gid_t ar_gid;			/* Group ID.  */
  mode_t ar_mode;		/* File mode.  */
  off_t ar_size;		/* File size.  */
  char *ar_rawname;		/* Original name of archive member.  */
} Elf_Arhdr;


/* Archive symbol table entry.  */
typedef struct
{
  char *as_name;		/* Symbol name.  */
  size_t as_off;		/* Offset for this file in the archive.  */
  unsigned long int as_hash;	/* Hash value of the name.  */
} Elf_Arsym;


/* Descriptor for the ELF file.  */
typedef struct Elf Elf;

/* Descriptor for ELF file section.  */
typedef struct Elf_Scn Elf_Scn;


#ifdef __cplusplus
extern "C" {
#endif

/* Return descriptor for ELF file to work according to CMD.  */
extern Elf *elf_begin (int __fildes, Elf_Cmd __cmd, Elf *__ref);

/* Create a clone of an existing ELF descriptor.  */
  extern Elf *elf_clone (Elf *__elf, Elf_Cmd __cmd);

/* Create descriptor for memory region.  */
extern Elf *elf_memory (char *__image, size_t __size);

/* Advance archive descriptor to next element.  */
extern Elf_Cmd elf_next (Elf *__elf);

/* Free resources allocated for ELF.  */
extern int elf_end (Elf *__elf);

/* Update ELF descriptor and write file to disk.  */
extern off_t elf_update (Elf *__elf, Elf_Cmd __cmd);

/* Determine what kind of file is associated with ELF.  */
extern Elf_Kind elf_kind (Elf *__elf) __attribute__ ((__pure__));

/* Get the base offset for an object file.  */
extern off_t elf_getbase (Elf *__elf);


/* Retrieve file identification data.  */
extern char *elf_getident (Elf *__elf, size_t *__ptr);

/* Retrieve class-dependent object file header.  */
extern Elf32_Ehdr *elf32_getehdr (Elf *__elf);
/* Similar but this time the binary calls is ELFCLASS64.  */
extern Elf64_Ehdr *elf64_getehdr (Elf *__elf);

/* Create ELF header if none exists.  */
extern Elf32_Ehdr *elf32_newehdr (Elf *__elf);
/* Similar but this time the binary calls is ELFCLASS64.  */
extern Elf64_Ehdr *elf64_newehdr (Elf *__elf);

/* Retrieve class-dependent program header table.  */
extern Elf32_Phdr *elf32_getphdr (Elf *__elf);
/* Similar but this time the binary calls is ELFCLASS64.  */
extern Elf64_Phdr *elf64_getphdr (Elf *__elf);

/* Create ELF program header.  */
extern Elf32_Phdr *elf32_newphdr (Elf *__elf, size_t __cnt);
/* Similar but this time the binary calls is ELFCLASS64.  */
extern Elf64_Phdr *elf64_newphdr (Elf *__elf, size_t __cnt);


/* Get section at INDEX.  */
extern Elf_Scn *elf_getscn (Elf *__elf, size_t __index);

/* Get index of section.  */
extern size_t elf_ndxscn (Elf_Scn *__scn);

/* Get section with next section index.  */
extern Elf_Scn *elf_nextscn (Elf *__elf, Elf_Scn *__scn);

/* Create a new section and append it at the end of the table.  */
extern Elf_Scn *elf_newscn (Elf *__elf);

extern int elf_getshnum (Elf *__elf, size_t *__dst);


extern int elf_getshstrndx (Elf *__elf, size_t *__dst);


/* Retrieve section header of ELFCLASS32 binary.  */
extern Elf32_Shdr *elf32_getshdr (Elf_Scn *__scn);
/* Similar for ELFCLASS64.  */
extern Elf64_Shdr *elf64_getshdr (Elf_Scn *__scn);


/* Set or clear flags for ELF file.  */
extern unsigned int elf_flagelf (Elf *__elf, Elf_Cmd __cmd,
				 unsigned int __flags);
/* Similarly for the ELF header.  */
extern unsigned int elf_flagehdr (Elf *__elf, Elf_Cmd __cmd,
				  unsigned int __flags);
/* Similarly for the ELF program header.  */
extern unsigned int elf_flagphdr (Elf *__elf, Elf_Cmd __cmd,
				  unsigned int __flags);
/* Similarly for the given ELF section.  */
extern unsigned int elf_flagscn (Elf_Scn *__scn, Elf_Cmd __cmd,
				 unsigned int __flags);
/* Similarly for the given ELF data.  */
extern unsigned int elf_flagdata (Elf_Data *__data, Elf_Cmd __cmd,
				  unsigned int __flags);
/* Similarly for the given ELF section header.  */
extern unsigned int elf_flagshdr (Elf_Scn *__scn, Elf_Cmd __cmd,
				  unsigned int __flags);


extern Elf_Data *elf_getdata (Elf_Scn *__scn, Elf_Data *__data);

/* Get uninterpreted section content.  */
extern Elf_Data *elf_rawdata (Elf_Scn *__scn, Elf_Data *__data);

/* Create new data descriptor for section SCN.  */
extern Elf_Data *elf_newdata (Elf_Scn *__scn);


/* Return pointer to string at OFFSET in section INDEX.  */
extern char *elf_strptr (Elf *__elf, size_t __index, size_t __offset);


/* Return header of archive.  */
extern Elf_Arhdr *elf_getarhdr (Elf *__elf);

/* Select archive element at OFFSET.  */
extern size_t elf_rand (Elf *__elf, size_t __offset);

/* Get symbol table of archhive.  */
extern Elf_Arsym *elf_getarsym (Elf *__elf, size_t *__ptr);


/* Control ELF descriptor.  */
extern int elf_cntl (Elf *__elf, Elf_Cmd __cmd);

/* Retrieve uninterpreted file contents.  */
extern char *elf_rawfile (Elf *__elf, size_t *__ptr);


extern size_t elf32_fsize (Elf_Type __type, size_t __count,
			   unsigned int __version)
       __attribute__ ((__const__));
/* Similar but this time the binary calls is ELFCLASS64.  */
extern size_t elf64_fsize (Elf_Type __type, size_t __count,
			   unsigned int __version)
       __attribute__ ((__const__));


extern Elf_Data *elf32_xlatetom (Elf_Data *__dest, const Elf_Data *__src,
				 unsigned int __encode);
/* Same for 64 bit class.  */
extern Elf_Data *elf64_xlatetom (Elf_Data *__dest, const Elf_Data *__src,
				 unsigned int __encode);

extern Elf_Data *elf32_xlatetof (Elf_Data *__dest, const Elf_Data *__src,
				 unsigned int __encode);
/* Same for 64 bit class.  */
extern Elf_Data *elf64_xlatetof (Elf_Data *__dest, const Elf_Data *__src,
				 unsigned int __encode);


extern int elf_errno (void);

extern const char *elf_errmsg (int __error);


/* Coordinate ELF library and application versions.  */
extern unsigned int elf_version (unsigned int __version);

/* Set fill bytes used to fill holes in data structures.  */
extern void elf_fill (int __fill);

/* Compute hash value.  */
extern unsigned long int elf_hash (const char *__string)
       __attribute__ ((__pure__));


/* Compute simple checksum from permanent parts of the ELF file.  */
extern long int elf32_checksum (Elf *__elf);
/* Similar but this time the binary calls is ELFCLASS64.  */
extern long int elf64_checksum (Elf *__elf);

#ifdef __cplusplus
}
#endif

#endif  /* libelf.h */
