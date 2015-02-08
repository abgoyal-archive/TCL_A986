
#ifndef _NLIST_H
#define _NLIST_H 1


/* Symbol list type.  */
struct nlist
{
  char *n_name;			/* Symbol name.  */
  long int n_value;		/* Value of symbol.  */
  short int n_scnum;		/* Section number found in.  */
  unsigned short int n_type;	/* Type of symbol.  */
  char n_sclass;		/* Storage class.  */
  char n_numaux;		/* Number of auxiliary entries.  */
};


#ifdef __cplusplus
extern "C" {
#endif

/* Get specified entries from file.  */
extern int nlist (__const char *__filename, struct nlist *__nl);

#ifdef __cplusplus
}
#endif

#endif  /* nlist.h */
