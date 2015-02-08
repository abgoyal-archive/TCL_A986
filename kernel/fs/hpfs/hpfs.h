


/* Notation */

typedef unsigned secno;			/* sector number, partition relative */

typedef secno dnode_secno;		/* sector number of a dnode */
typedef secno fnode_secno;		/* sector number of an fnode */
typedef secno anode_secno;		/* sector number of an anode */

typedef u32 time32_t;		/* 32-bit time_t type */

/* sector 0 */


#define BB_MAGIC 0xaa55

struct hpfs_boot_block
{
  unsigned char jmp[3];
  unsigned char oem_id[8];
  unsigned char bytes_per_sector[2];	/* 512 */
  unsigned char sectors_per_cluster;
  unsigned char n_reserved_sectors[2];
  unsigned char n_fats;
  unsigned char n_rootdir_entries[2];
  unsigned char n_sectors_s[2];
  unsigned char media_byte;
  unsigned short sectors_per_fat;
  unsigned short sectors_per_track;
  unsigned short heads_per_cyl;
  unsigned int n_hidden_sectors;
  unsigned int n_sectors_l;		/* size of partition */
  unsigned char drive_number;
  unsigned char mbz;
  unsigned char sig_28h;		/* 28h */
  unsigned char vol_serno[4];
  unsigned char vol_label[11];
  unsigned char sig_hpfs[8];		/* "HPFS    " */
  unsigned char pad[448];
  unsigned short magic;			/* aa55 */
};


/* sector 16 */

/* The super block has the pointer to the root directory. */

#define SB_MAGIC 0xf995e849

struct hpfs_super_block
{
  unsigned magic;			/* f995 e849 */
  unsigned magic1;			/* fa53 e9c5, more magic? */
  /*unsigned huh202;*/			/* ?? 202 = N. of B. in 1.00390625 S.*/
  char version;				/* version of a filesystem  usually 2 */
  char funcversion;			/* functional version - oldest version
  					   of filesystem that can understand
					   this disk */
  unsigned short int zero;		/* 0 */
  fnode_secno root;			/* fnode of root directory */
  secno n_sectors;			/* size of filesystem */
  unsigned n_badblocks;			/* number of bad blocks */
  secno bitmaps;			/* pointers to free space bit maps */
  unsigned zero1;			/* 0 */
  secno badblocks;			/* bad block list */
  unsigned zero3;			/* 0 */
  time32_t last_chkdsk;			/* date last checked, 0 if never */
  /*unsigned zero4;*/			/* 0 */
  time32_t last_optimize;			/* date last optimized, 0 if never */
  secno n_dir_band;			/* number of sectors in dir band */
  secno dir_band_start;			/* first sector in dir band */
  secno dir_band_end;			/* last sector in dir band */
  secno dir_band_bitmap;		/* free space map, 1 dnode per bit */
  char volume_name[32];			/* not used */
  secno user_id_table;			/* 8 preallocated sectors - user id */
  unsigned zero6[103];			/* 0 */
};


/* sector 17 */

/* The spare block has pointers to spare sectors.  */

#define SP_MAGIC 0xf9911849

struct hpfs_spare_block
{
  unsigned magic;			/* f991 1849 */
  unsigned magic1;			/* fa52 29c5, more magic? */

  unsigned dirty: 1;			/* 0 clean, 1 "improperly stopped" */
  /*unsigned flag1234: 4;*/		/* unknown flags */
  unsigned sparedir_used: 1;		/* spare dirblks used */
  unsigned hotfixes_used: 1;		/* hotfixes used */
  unsigned bad_sector: 1;		/* bad sector, corrupted disk (???) */
  unsigned bad_bitmap: 1;		/* bad bitmap */
  unsigned fast: 1;			/* partition was fast formatted */
  unsigned old_wrote: 1;		/* old version wrote to partion */
  unsigned old_wrote_1: 1;		/* old version wrote to partion (?) */
  unsigned install_dasd_limits: 1;	/* HPFS386 flags */
  unsigned resynch_dasd_limits: 1;
  unsigned dasd_limits_operational: 1;
  unsigned multimedia_active: 1;
  unsigned dce_acls_active: 1;
  unsigned dasd_limits_dirty: 1;
  unsigned flag67: 2;
  unsigned char mm_contlgulty;
  unsigned char unused;

  secno hotfix_map;			/* info about remapped bad sectors */
  unsigned n_spares_used;		/* number of hotfixes */
  unsigned n_spares;			/* number of spares in hotfix map */
  unsigned n_dnode_spares_free;		/* spare dnodes unused */
  unsigned n_dnode_spares;		/* length of spare_dnodes[] list,
					   follows in this block*/
  secno code_page_dir;			/* code page directory block */
  unsigned n_code_pages;		/* number of code pages */
  /*unsigned large_numbers[2];*/	/* ?? */
  unsigned super_crc;			/* on HPFS386 and LAN Server this is
  					   checksum of superblock, on normal
					   OS/2 unused */
  unsigned spare_crc;			/* on HPFS386 checksum of spareblock */
  unsigned zero1[15];			/* unused */
  dnode_secno spare_dnodes[100];	/* emergency free dnode list */
  unsigned zero2[1];			/* room for more? */
};


#define BAD_MAGIC 0
       





/* block pointed to by spareblock->code_page_dir */

#define CP_DIR_MAGIC 0x494521f7

struct code_page_directory
{
  unsigned magic;			/* 4945 21f7 */
  unsigned n_code_pages;		/* number of pointers following */
  unsigned zero1[2];
  struct {
    unsigned short ix;			/* index */
    unsigned short code_page_number;	/* code page number */
    unsigned bounds;			/* matches corresponding word
					   in data block */
    secno code_page_data;		/* sector number of a code_page_data
					   containing c.p. array */
    unsigned short index;		/* index in c.p. array in that sector*/
    unsigned short unknown;		/* some unknown value; usually 0;
    					   2 in Japanese version */
  } array[31];				/* unknown length */
};

/* blocks pointed to by code_page_directory */

#define CP_DATA_MAGIC 0x894521f7

struct code_page_data
{
  unsigned magic;			/* 8945 21f7 */
  unsigned n_used;			/* # elements used in c_p_data[] */
  unsigned bounds[3];			/* looks a bit like
					     (beg1,end1), (beg2,end2)
					   one byte each */
  unsigned short offs[3];		/* offsets from start of sector
					   to start of c_p_data[ix] */
  struct {
    unsigned short ix;			/* index */
    unsigned short code_page_number;	/* code page number */
    unsigned short unknown;		/* the same as in cp directory */
    unsigned char map[128];		/* upcase table for chars 80..ff */
    unsigned short zero2;
  } code_page[3];
  unsigned char incognita[78];
};




/* dnode: directory.  4 sectors long */


#define DNODE_MAGIC   0x77e40aae

struct dnode {
  unsigned magic;			/* 77e4 0aae */
  unsigned first_free;			/* offset from start of dnode to
					   first free dir entry */
  unsigned root_dnode:1;		/* Is it root dnode? */
  unsigned increment_me:31;		/* some kind of activity counter?
					   Neither HPFS.IFS nor CHKDSK cares
					   if you change this word */
  secno up;				/* (root dnode) directory's fnode
					   (nonroot) parent dnode */
  dnode_secno self;			/* pointer to this dnode */
  unsigned char dirent[2028];		/* one or more dirents */
};

struct hpfs_dirent {
  unsigned short length;		/* offset to next dirent */
  unsigned first: 1;			/* set on phony ^A^A (".") entry */
  unsigned has_acl: 1;
  unsigned down: 1;			/* down pointer present (after name) */
  unsigned last: 1;			/* set on phony \377 entry */
  unsigned has_ea: 1;			/* entry has EA */
  unsigned has_xtd_perm: 1;		/* has extended perm list (???) */
  unsigned has_explicit_acl: 1;
  unsigned has_needea: 1;		/* ?? some EA has NEEDEA set
					   I have no idea why this is
					   interesting in a dir entry */
  unsigned read_only: 1;		/* dos attrib */
  unsigned hidden: 1;			/* dos attrib */
  unsigned system: 1;			/* dos attrib */
  unsigned flag11: 1;			/* would be volume label dos attrib */
  unsigned directory: 1;		/* dos attrib */
  unsigned archive: 1;			/* dos attrib */
  unsigned not_8x3: 1;			/* name is not 8.3 */
  unsigned flag15: 1;
  fnode_secno fnode;			/* fnode giving allocation info */
  time32_t write_date;			/* mtime */
  unsigned file_size;			/* file length, bytes */
  time32_t read_date;			/* atime */
  time32_t creation_date;			/* ctime */
  unsigned ea_size;			/* total EA length, bytes */
  unsigned char no_of_acls : 3;		/* number of ACL's */
  unsigned char reserver : 5;
  unsigned char ix;			/* code page index (of filename), see
					   struct code_page_data */
  unsigned char namelen, name[1];	/* file name */
  /* dnode_secno down;	  btree down pointer, if present,
     			  follows name on next word boundary, or maybe it
			  precedes next dirent, which is on a word boundary. */
};


/* B+ tree: allocation info in fnodes and anodes */


struct bplus_leaf_node
{
  unsigned file_secno;			/* first file sector in extent */
  unsigned length;			/* length, sectors */
  secno disk_secno;			/* first corresponding disk sector */
};

struct bplus_internal_node
{
  unsigned file_secno;			/* subtree maps sectors < this  */
  anode_secno down;			/* pointer to subtree */
};

struct bplus_header
{
  unsigned hbff: 1;	/* high bit of first free entry offset */
  unsigned flag1: 1;
  unsigned flag2: 1;
  unsigned flag3: 1;
  unsigned flag4: 1;
  unsigned fnode_parent: 1;		/* ? we're pointed to by an fnode,
					   the data btree or some ea or the
					   main ea bootage pointer ea_secno */
					/* also can get set in fnodes, which
					   may be a chkdsk glitch or may mean
					   this bit is irrelevant in fnodes,
					   or this interpretation is all wet */
  unsigned binary_search: 1;		/* suggest binary search (unused) */
  unsigned internal: 1;			/* 1 -> (internal) tree of anodes
					   0 -> (leaf) list of extents */
  unsigned char fill[3];
  unsigned char n_free_nodes;		/* free nodes in following array */
  unsigned char n_used_nodes;		/* used nodes in following array */
  unsigned short first_free;		/* offset from start of header to
					   first free node in array */
  union {
    struct bplus_internal_node internal[0]; /* (internal) 2-word entries giving
					       subtree pointers */
    struct bplus_leaf_node external[0];	    /* (external) 3-word entries giving
					       sector runs */
  } u;
};

/* fnode: root of allocation b+ tree, and EA's */


#define FNODE_MAGIC 0xf7e40aae

struct fnode
{
  unsigned magic;			/* f7e4 0aae */
  unsigned zero1[2];			/* read history */
  unsigned char len, name[15];		/* true length, truncated name */
  fnode_secno up;			/* pointer to file's directory fnode */
  /*unsigned zero2[3];*/
  secno acl_size_l;
  secno acl_secno;
  unsigned short acl_size_s;
  char acl_anode;
  char zero2;				/* history bit count */
  unsigned ea_size_l;			/* length of disk-resident ea's */
  secno ea_secno;			/* first sector of disk-resident ea's*/
  unsigned short ea_size_s;		/* length of fnode-resident ea's */

  unsigned flag0: 1;
  unsigned ea_anode: 1;			/* 1 -> ea_secno is an anode */
  unsigned flag2: 1;
  unsigned flag3: 1;
  unsigned flag4: 1;
  unsigned flag5: 1;
  unsigned flag6: 1;
  unsigned flag7: 1;
  unsigned dirflag: 1;			/* 1 -> directory.  first & only extent
					   points to dnode. */
  unsigned flag9: 1;
  unsigned flag10: 1;
  unsigned flag11: 1;
  unsigned flag12: 1;
  unsigned flag13: 1;
  unsigned flag14: 1;
  unsigned flag15: 1;

  struct bplus_header btree;		/* b+ tree, 8 extents or 12 subtrees */
  union {
    struct bplus_leaf_node external[8];
    struct bplus_internal_node internal[12];
  } u;

  unsigned file_size;			/* file length, bytes */
  unsigned n_needea;			/* number of EA's with NEEDEA set */
  char user_id[16];			/* unused */
  unsigned short ea_offs;		/* offset from start of fnode
					   to first fnode-resident ea */
  char dasd_limit_treshhold;
  char dasd_limit_delta;
  unsigned dasd_limit;
  unsigned dasd_usage;
  /*unsigned zero5[2];*/
  unsigned char ea[316];		/* zero or more EA's, packed together
					   with no alignment padding.
					   (Do not use this name, get here
					   via fnode + ea_offs. I think.) */
};


/* anode: 99.44% pure allocation tree */

#define ANODE_MAGIC 0x37e40aae

struct anode
{
  unsigned magic;			/* 37e4 0aae */
  anode_secno self;			/* pointer to this anode */
  secno up;				/* parent anode or fnode */

  struct bplus_header btree;		/* b+tree, 40 extents or 60 subtrees */
  union {
    struct bplus_leaf_node external[40];
    struct bplus_internal_node internal[60];
  } u;

  unsigned fill[3];			/* unused */
};



struct extended_attribute
{
  unsigned indirect: 1;			/* 1 -> value gives sector number
					   where real value starts */
  unsigned anode: 1;			/* 1 -> sector is an anode
					   that points to fragmented value */
  unsigned flag2: 1;
  unsigned flag3: 1;
  unsigned flag4: 1;
  unsigned flag5: 1;
  unsigned flag6: 1;
  unsigned needea: 1;			/* required ea */
  unsigned char namelen;		/* length of name, bytes */
  unsigned short valuelen;		/* length of value, bytes */
  unsigned char name[0];
  /*
    unsigned char name[namelen];	ascii attrib name
    unsigned char nul;			terminating '\0', not counted
    unsigned char value[valuelen];	value, arbitrary
      if this.indirect, valuelen is 8 and the value is
        unsigned length;		real length of value, bytes
        secno secno;			sector address where it starts
      if this.anode, the above sector number is the root of an anode tree
        which points to the value.
  */
};

