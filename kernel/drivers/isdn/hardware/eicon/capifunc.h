

#ifndef __CAPIFUNC_H__
#define __CAPIFUNC_H__

#define DRRELMAJOR  2
#define DRRELMINOR  0
#define DRRELEXTRA  ""

#define M_COMPANY "Eicon Networks"

extern char DRIVERRELEASE_CAPI[];

typedef struct _diva_card {
	struct list_head list;
	int remove_in_progress;
	int Id;
	struct capi_ctr capi_ctrl;
	DIVA_CAPI_ADAPTER *adapter;
	DESCRIPTOR d;
	char name[32];
} diva_card;

int init_capifunc(void);
void finit_capifunc(void);

#endif /* __CAPIFUNC_H__ */
