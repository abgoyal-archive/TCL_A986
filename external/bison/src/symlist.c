
#include <config.h>
#include "system.h"

#include "complain.h"
#include "symlist.h"



symbol_list *
symbol_list_new (symbol *sym, location loc)
{
  symbol_list *res = xmalloc (sizeof *res);

  res->sym = sym;
  res->location = loc;

  res->midrule = NULL;

  res->action = NULL;
  res->used = false;

  res->ruleprec = NULL;
  res->dprec = 0;
  res->merger = 0;

  res->next = NULL;

  return res;
}



void
symbol_list_print (const symbol_list *l, FILE *f)
{
  for (/* Nothing. */; l && l->sym; l = l->next)
    {
      symbol_print (l->sym, f);
      fprintf (stderr, l->used ? " used" : " unused");
      if (l && l->sym)
	fprintf (f, ", ");
    }
}



symbol_list *
symbol_list_prepend (symbol_list *list, symbol *sym, location loc)
{
  symbol_list *res = symbol_list_new (sym, loc);
  res->next = list;
  return res;
}



void
symbol_list_free (symbol_list *list)
{
  LIST_FREE (symbol_list, list);
}



unsigned int
symbol_list_length (const symbol_list *l)
{
  int res = 0;
  for (/* Nothing. */; l; l = l->next)
    ++res;
  return res;
}



symbol_list *
symbol_list_n_get (symbol_list *l, int n)
{
  int i;

  if (n < 0)
    return NULL;

  for (i = 0; i < n; ++i)
    {
      l = l->next;
      if (l == NULL || l->sym == NULL)
	return NULL;
    }

  return l;
}



uniqstr
symbol_list_n_type_name_get (symbol_list *l, location loc, int n)
{
  l = symbol_list_n_get (l, n);
  if (!l)
    {
      complain_at (loc, _("invalid $ value: $%d"), n);
      return NULL;
    }
  return l->sym->type_name;
}



void
symbol_list_n_used_set (symbol_list *l, int n, bool used)
{
  l = symbol_list_n_get (l, n);
  if (l)
    l->used = used;
}
