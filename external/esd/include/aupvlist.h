

#ifndef AUPVLIST_H
#define AUPVLIST_H

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

enum
{
	AU_PVTYPE_LONG = 1,
	AU_PVTYPE_DOUBLE = 2,
	AU_PVTYPE_PTR = 3
};

typedef struct _AUpvlist *AUpvlist;

#define AU_NULL_PVLIST ((struct _AUpvlist *) 0)

AUpvlist AUpvnew (int maxItems);
int AUpvgetmaxitems (AUpvlist);
int AUpvfree (AUpvlist);
int AUpvsetparam (AUpvlist, int item, int param);
int AUpvsetvaltype (AUpvlist, int item, int type);
int AUpvsetval (AUpvlist, int item, void *val);
int AUpvgetparam (AUpvlist, int item, int *param);
int AUpvgetvaltype (AUpvlist, int item, int *type);
int AUpvgetval (AUpvlist, int item, void *val);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* AUPVLIST_H */
