

#ifndef __XEN_PUBLIC_GRANT_TABLE_H__
#define __XEN_PUBLIC_GRANT_TABLE_H__




struct grant_entry {
    /* GTF_xxx: various type and flag information.  [XEN,GST] */
    uint16_t flags;
    /* The domain being granted foreign privileges. [GST] */
    domid_t  domid;
    /*
     * GTF_permit_access: Frame that @domid is allowed to map and access. [GST]
     * GTF_accept_transfer: Frame whose ownership transferred by @domid. [XEN]
     */
    uint32_t frame;
};

#define GTF_invalid         (0U<<0)
#define GTF_permit_access   (1U<<0)
#define GTF_accept_transfer (2U<<0)
#define GTF_type_mask       (3U<<0)

#define _GTF_readonly       (2)
#define GTF_readonly        (1U<<_GTF_readonly)
#define _GTF_reading        (3)
#define GTF_reading         (1U<<_GTF_reading)
#define _GTF_writing        (4)
#define GTF_writing         (1U<<_GTF_writing)

#define _GTF_transfer_committed (2)
#define GTF_transfer_committed  (1U<<_GTF_transfer_committed)
#define _GTF_transfer_completed (3)
#define GTF_transfer_completed  (1U<<_GTF_transfer_completed)



typedef uint32_t grant_ref_t;

typedef uint32_t grant_handle_t;

#define GNTTABOP_map_grant_ref        0
struct gnttab_map_grant_ref {
    /* IN parameters. */
    uint64_t host_addr;
    uint32_t flags;               /* GNTMAP_* */
    grant_ref_t ref;
    domid_t  dom;
    /* OUT parameters. */
    int16_t  status;              /* GNTST_* */
    grant_handle_t handle;
    uint64_t dev_bus_addr;
};
DEFINE_GUEST_HANDLE_STRUCT(gnttab_map_grant_ref);

#define GNTTABOP_unmap_grant_ref      1
struct gnttab_unmap_grant_ref {
    /* IN parameters. */
    uint64_t host_addr;
    uint64_t dev_bus_addr;
    grant_handle_t handle;
    /* OUT parameters. */
    int16_t  status;              /* GNTST_* */
};
DEFINE_GUEST_HANDLE_STRUCT(gnttab_unmap_grant_ref);

#define GNTTABOP_setup_table          2
struct gnttab_setup_table {
    /* IN parameters. */
    domid_t  dom;
    uint32_t nr_frames;
    /* OUT parameters. */
    int16_t  status;              /* GNTST_* */
    GUEST_HANDLE(ulong) frame_list;
};
DEFINE_GUEST_HANDLE_STRUCT(gnttab_setup_table);

#define GNTTABOP_dump_table           3
struct gnttab_dump_table {
    /* IN parameters. */
    domid_t dom;
    /* OUT parameters. */
    int16_t status;               /* GNTST_* */
};
DEFINE_GUEST_HANDLE_STRUCT(gnttab_dump_table);

#define GNTTABOP_transfer                4
struct gnttab_transfer {
    /* IN parameters. */
    unsigned long mfn;
    domid_t       domid;
    grant_ref_t   ref;
    /* OUT parameters. */
    int16_t       status;
};
DEFINE_GUEST_HANDLE_STRUCT(gnttab_transfer);


#define _GNTCOPY_source_gref      (0)
#define GNTCOPY_source_gref       (1<<_GNTCOPY_source_gref)
#define _GNTCOPY_dest_gref        (1)
#define GNTCOPY_dest_gref         (1<<_GNTCOPY_dest_gref)

#define GNTTABOP_copy                 5
struct gnttab_copy {
	/* IN parameters. */
	struct {
		union {
			grant_ref_t ref;
			unsigned long   gmfn;
		} u;
		domid_t  domid;
		uint16_t offset;
	} source, dest;
	uint16_t      len;
	uint16_t      flags;          /* GNTCOPY_* */
	/* OUT parameters. */
	int16_t       status;
};
DEFINE_GUEST_HANDLE_STRUCT(gnttab_copy);

#define GNTTABOP_query_size           6
struct gnttab_query_size {
    /* IN parameters. */
    domid_t  dom;
    /* OUT parameters. */
    uint32_t nr_frames;
    uint32_t max_nr_frames;
    int16_t  status;              /* GNTST_* */
};
DEFINE_GUEST_HANDLE_STRUCT(gnttab_query_size);

 /* Map the grant entry for access by I/O devices. */
#define _GNTMAP_device_map      (0)
#define GNTMAP_device_map       (1<<_GNTMAP_device_map)
 /* Map the grant entry for access by host CPUs. */
#define _GNTMAP_host_map        (1)
#define GNTMAP_host_map         (1<<_GNTMAP_host_map)
 /* Accesses to the granted frame will be restricted to read-only access. */
#define _GNTMAP_readonly        (2)
#define GNTMAP_readonly         (1<<_GNTMAP_readonly)
 /*
  * GNTMAP_host_map subflag:
  *  0 => The host mapping is usable only by the guest OS.
  *  1 => The host mapping is usable by guest OS + current application.
  */
#define _GNTMAP_application_map (3)
#define GNTMAP_application_map  (1<<_GNTMAP_application_map)

 /*
  * GNTMAP_contains_pte subflag:
  *  0 => This map request contains a host virtual address.
  *  1 => This map request contains the machine addess of the PTE to update.
  */
#define _GNTMAP_contains_pte    (4)
#define GNTMAP_contains_pte     (1<<_GNTMAP_contains_pte)

#define GNTST_okay             (0)  /* Normal return.                        */
#define GNTST_general_error    (-1) /* General undefined error.              */
#define GNTST_bad_domain       (-2) /* Unrecognsed domain id.                */
#define GNTST_bad_gntref       (-3) /* Unrecognised or inappropriate gntref. */
#define GNTST_bad_handle       (-4) /* Unrecognised or inappropriate handle. */
#define GNTST_bad_virt_addr    (-5) /* Inappropriate virtual address to map. */
#define GNTST_bad_dev_addr     (-6) /* Inappropriate device address to unmap.*/
#define GNTST_no_device_space  (-7) /* Out of space in I/O MMU.              */
#define GNTST_permission_denied (-8) /* Not enough privilege for operation.  */
#define GNTST_bad_page         (-9) /* Specified page was invalid for op.    */
#define GNTST_bad_copy_arg    (-10) /* copy arguments cross page boundary */

#define GNTTABOP_error_msgs {                   \
    "okay",                                     \
    "undefined error",                          \
    "unrecognised domain id",                   \
    "invalid grant reference",                  \
    "invalid mapping handle",                   \
    "invalid virtual address",                  \
    "invalid device address",                   \
    "no spare translation slot in the I/O MMU", \
    "permission denied",                        \
    "bad page",                                 \
    "copy arguments cross page boundary"        \
}

#endif /* __XEN_PUBLIC_GRANT_TABLE_H__ */
