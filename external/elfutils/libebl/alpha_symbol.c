
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <elf.h>
#include <stddef.h>

#include <libebl_alpha.h>


/* Return of the backend.  */
const char *
alpha_backend_name (void)
{
  return "alpha";
}


/* Relocation mapping table.  */
static const char *reloc_map_table[] =
  {
    [R_ALPHA_NONE] = "R_ALPHA_NONE",
    [R_ALPHA_REFLONG] = "R_ALPHA_REFLONG",
    [R_ALPHA_REFQUAD] = "R_ALPHA_REFQUAD",
    [R_ALPHA_GPREL32] = "R_ALPHA_GPREL32",
    [R_ALPHA_LITERAL] = "R_ALPHA_LITERAL",
    [R_ALPHA_LITUSE] = "R_ALPHA_LITUSE",
    [R_ALPHA_GPDISP] = "R_ALPHA_GPDISP",
    [R_ALPHA_BRADDR] = "R_ALPHA_BRADDR",
    [R_ALPHA_HINT] = "R_ALPHA_HINT",
    [R_ALPHA_SREL16] = "R_ALPHA_SREL16",
    [R_ALPHA_SREL32] = "R_ALPHA_SREL32",
    [R_ALPHA_SREL64] = "R_ALPHA_SREL64",
    [R_ALPHA_GPRELHIGH] = "R_ALPHA_GPRELHIGH",
    [R_ALPHA_GPRELLOW] = "R_ALPHA_GPRELLOW",
    [R_ALPHA_GPREL16] = "R_ALPHA_GPREL16",
    [R_ALPHA_COPY] = "R_ALPHA_COPY",
    [R_ALPHA_GLOB_DAT] = "R_ALPHA_GLOB_DAT",
    [R_ALPHA_JMP_SLOT] = "R_ALPHA_JMP_SLOT",
    [R_ALPHA_RELATIVE] = "R_ALPHA_RELATIVE",
    [R_ALPHA_TLS_GD_HI] = "R_ALPHA_TLS_GD_HI",
    [R_ALPHA_TLSGD] = "R_ALPHA_TLSGD",
    [R_ALPHA_TLS_LDM] = "R_ALPHA_TLS_LDM",
    [R_ALPHA_DTPMOD64] = "R_ALPHA_DTPMOD64",
    [R_ALPHA_GOTDTPREL] = "R_ALPHA_GOTDTPREL",
    [R_ALPHA_DTPREL64] = "R_ALPHA_DTPREL64",
    [R_ALPHA_DTPRELHI] = "R_ALPHA_DTPRELHI",
    [R_ALPHA_DTPRELLO] = "R_ALPHA_DTPRELLO",
    [R_ALPHA_DTPREL16] = "R_ALPHA_DTPREL16",
    [R_ALPHA_GOTTPREL] = "R_ALPHA_GOTTPREL",
    [R_ALPHA_TPREL64] = "R_ALPHA_TPREL64",
    [R_ALPHA_TPRELHI] = "R_ALPHA_TPRELHI",
    [R_ALPHA_TPRELLO] = "R_ALPHA_TPRELLO",
    [R_ALPHA_TPREL16] = "R_ALPHA_TPREL16"
  };


/* Determine relocation type string for Alpha.  */
const char *
alpha_reloc_type_name (int type, char *buf, size_t len)
{
  if (type < 0
      || ((size_t) type
	  >= sizeof (reloc_map_table) / sizeof (reloc_map_table[0])))
    return NULL;

  return reloc_map_table[type];
}


/* Check for correct relocation type.  */
bool
alpha_reloc_type_check (int type)
{
  return (type >= R_ALPHA_NONE
	  && ((size_t) type
	      < sizeof (reloc_map_table) / sizeof (reloc_map_table[0]))
	  && reloc_map_table[type] != NULL) ? true : false;
}
