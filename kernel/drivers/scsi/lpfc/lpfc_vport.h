

#ifndef _H_LPFC_VPORT
#define _H_LPFC_VPORT

/* API version values (each will be an individual bit) */
#define VPORT_API_VERSION_1	0x01

/* Values returned via lpfc_vport_getinfo() */
struct vport_info {

	uint32_t api_versions;
	uint8_t linktype;
#define  VPORT_TYPE_PHYSICAL	0
#define  VPORT_TYPE_VIRTUAL	1

	uint8_t state;
#define  VPORT_STATE_OFFLINE	0
#define  VPORT_STATE_ACTIVE	1
#define  VPORT_STATE_FAILED	2

	uint8_t fail_reason;
	uint8_t prev_fail_reason;
#define  VPORT_FAIL_UNKNOWN	0
#define  VPORT_FAIL_LINKDOWN	1
#define  VPORT_FAIL_FAB_UNSUPPORTED	2
#define  VPORT_FAIL_FAB_NORESOURCES	3
#define  VPORT_FAIL_FAB_LOGOUT	4
#define  VPORT_FAIL_ADAP_NORESOURCES	5

	uint8_t node_name[8];	/* WWNN */
	uint8_t port_name[8];	/* WWPN */

	struct Scsi_Host *shost;

/* Following values are valid only on physical links */
	uint32_t vports_max;
	uint32_t vports_inuse;
	uint32_t rpi_max;
	uint32_t rpi_inuse;
#define  VPORT_CNT_INVALID	0xFFFFFFFF
};

/* data used  in link creation */
struct vport_data {
	uint32_t api_version;

	uint32_t options;
#define  VPORT_OPT_AUTORETRY	0x01

	uint8_t node_name[8];	/* WWNN */
	uint8_t port_name[8];	/* WWPN */

	struct Scsi_Host *vport_shost;
};

/* API function return codes */
#define VPORT_OK	0
#define VPORT_ERROR	-1
#define VPORT_INVAL	-2
#define VPORT_NOMEM	-3
#define VPORT_NORESOURCES	-4

int lpfc_vport_create(struct fc_vport *, bool);
int lpfc_vport_delete(struct fc_vport *);
int lpfc_vport_getinfo(struct Scsi_Host *, struct vport_info *);
int lpfc_vport_tgt_remove(struct Scsi_Host *, uint, uint);
struct lpfc_vport **lpfc_create_vport_work_array(struct lpfc_hba *);
void lpfc_destroy_vport_work_array(struct lpfc_hba *, struct lpfc_vport **);

#define  DID_VPORT_ERROR	0x0f

#define VPORT_INFO	0x1
#define VPORT_CREATE	0x2
#define VPORT_DELETE	0x4

struct vport_cmd_tag {
	uint32_t cmd;
	struct vport_data cdata;
	struct vport_info cinfo;
	void *vport;
	int vport_num;
};

void lpfc_vport_set_state(struct lpfc_vport *vport,
			  enum fc_vport_state new_state);

void lpfc_vport_reset_stat_data(struct lpfc_vport *);
void lpfc_alloc_bucket(struct lpfc_vport *);
void lpfc_free_bucket(struct lpfc_vport *);

#endif /* H_LPFC_VPORT */
