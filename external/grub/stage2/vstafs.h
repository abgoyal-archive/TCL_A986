

#ifndef VSTAFS_H
#define VSTAFS_H	1


#define LINE			16
#define BLOCK_SIZE		512
#define VSTAFS_START_DATA	320

struct bootrecord
{
  unsigned char flag;
  unsigned char s_sector;
  unsigned char s_head;
  unsigned char s_cylinder;
  unsigned char p_type;
  unsigned char e_sector;
  unsigned char e_head;
  unsigned char e_cylinder;
  unsigned long start_lba;
  unsigned long nr_sector_lba;
};

struct alloc
{
  unsigned long a_start;
  unsigned long a_len;
};

struct first_sector
{
  unsigned long fs_magic;
  unsigned long fs_size;
  unsigned long fs_extsize;
  unsigned long fs_free;
  struct  alloc fs_freesecs[0];
};

struct prot
{
  unsigned char len;
  unsigned char pdefault;
  unsigned char id[7];
  unsigned char bits[7];
};

struct fs_file
{
  unsigned long prev;
  unsigned long rev;
  unsigned long len;
  unsigned short type;
  unsigned short nlink;
  struct prot pprot;
  unsigned int owner;
  unsigned int extents;
  struct alloc blocks[32];
  long fs_ctime, fs_mtime; /* it is not lon but time_t */
  char pad[16];
  char data[0];
};

struct dir_entry
{
  char name[28];
  unsigned long start;
};

#endif /* ! VSTAFS_H */
