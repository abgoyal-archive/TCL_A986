

#include <rdma/ib_smi.h>
#include "smi.h"

enum smi_action smi_handle_dr_smp_send(struct ib_smp *smp,
				       u8 node_type, int port_num)
{
	u8 hop_ptr, hop_cnt;

	hop_ptr = smp->hop_ptr;
	hop_cnt = smp->hop_cnt;

	/* See section 14.2.2.2, Vol 1 IB spec */
	/* C14-6 -- valid hop_cnt values are from 0 to 63 */
	if (hop_cnt >= IB_SMP_MAX_PATH_HOPS)
		return IB_SMI_DISCARD;

	if (!ib_get_smp_direction(smp)) {
		/* C14-9:1 */
		if (hop_cnt && hop_ptr == 0) {
			smp->hop_ptr++;
			return (smp->initial_path[smp->hop_ptr] ==
				port_num ? IB_SMI_HANDLE : IB_SMI_DISCARD);
		}

		/* C14-9:2 */
		if (hop_ptr && hop_ptr < hop_cnt) {
			if (node_type != RDMA_NODE_IB_SWITCH)
				return IB_SMI_DISCARD;

			/* smp->return_path set when received */
			smp->hop_ptr++;
			return (smp->initial_path[smp->hop_ptr] ==
				port_num ? IB_SMI_HANDLE : IB_SMI_DISCARD);
		}

		/* C14-9:3 -- We're at the end of the DR segment of path */
		if (hop_ptr == hop_cnt) {
			/* smp->return_path set when received */
			smp->hop_ptr++;
			return (node_type == RDMA_NODE_IB_SWITCH ||
				smp->dr_dlid == IB_LID_PERMISSIVE ?
				IB_SMI_HANDLE : IB_SMI_DISCARD);
		}

		/* C14-9:4 -- hop_ptr = hop_cnt + 1 -> give to SMA/SM */
		/* C14-9:5 -- Fail unreasonable hop pointer */
		return (hop_ptr == hop_cnt + 1 ? IB_SMI_HANDLE : IB_SMI_DISCARD);

	} else {
		/* C14-13:1 */
		if (hop_cnt && hop_ptr == hop_cnt + 1) {
			smp->hop_ptr--;
			return (smp->return_path[smp->hop_ptr] ==
				port_num ? IB_SMI_HANDLE : IB_SMI_DISCARD);
		}

		/* C14-13:2 */
		if (2 <= hop_ptr && hop_ptr <= hop_cnt) {
			if (node_type != RDMA_NODE_IB_SWITCH)
				return IB_SMI_DISCARD;

			smp->hop_ptr--;
			return (smp->return_path[smp->hop_ptr] ==
				port_num ? IB_SMI_HANDLE : IB_SMI_DISCARD);
		}

		/* C14-13:3 -- at the end of the DR segment of path */
		if (hop_ptr == 1) {
			smp->hop_ptr--;
			/* C14-13:3 -- SMPs destined for SM shouldn't be here */
			return (node_type == RDMA_NODE_IB_SWITCH ||
				smp->dr_slid == IB_LID_PERMISSIVE ?
				IB_SMI_HANDLE : IB_SMI_DISCARD);
		}

		/* C14-13:4 -- hop_ptr = 0 -> should have gone to SM */
		if (hop_ptr == 0)
			return IB_SMI_HANDLE;

		/* C14-13:5 -- Check for unreasonable hop pointer */
		return IB_SMI_DISCARD;
	}
}

enum smi_action smi_handle_dr_smp_recv(struct ib_smp *smp, u8 node_type,
				       int port_num, int phys_port_cnt)
{
	u8 hop_ptr, hop_cnt;

	hop_ptr = smp->hop_ptr;
	hop_cnt = smp->hop_cnt;

	/* See section 14.2.2.2, Vol 1 IB spec */
	/* C14-6 -- valid hop_cnt values are from 0 to 63 */
	if (hop_cnt >= IB_SMP_MAX_PATH_HOPS)
		return IB_SMI_DISCARD;

	if (!ib_get_smp_direction(smp)) {
		/* C14-9:1 -- sender should have incremented hop_ptr */
		if (hop_cnt && hop_ptr == 0)
			return IB_SMI_DISCARD;

		/* C14-9:2 -- intermediate hop */
		if (hop_ptr && hop_ptr < hop_cnt) {
			if (node_type != RDMA_NODE_IB_SWITCH)
				return IB_SMI_DISCARD;

			smp->return_path[hop_ptr] = port_num;
			/* smp->hop_ptr updated when sending */
			return (smp->initial_path[hop_ptr+1] <= phys_port_cnt ?
				IB_SMI_HANDLE : IB_SMI_DISCARD);
		}

		/* C14-9:3 -- We're at the end of the DR segment of path */
		if (hop_ptr == hop_cnt) {
			if (hop_cnt)
				smp->return_path[hop_ptr] = port_num;
			/* smp->hop_ptr updated when sending */

			return (node_type == RDMA_NODE_IB_SWITCH ||
				smp->dr_dlid == IB_LID_PERMISSIVE ?
				IB_SMI_HANDLE : IB_SMI_DISCARD);
		}

		/* C14-9:4 -- hop_ptr = hop_cnt + 1 -> give to SMA/SM */
		/* C14-9:5 -- fail unreasonable hop pointer */
		return (hop_ptr == hop_cnt + 1 ? IB_SMI_HANDLE : IB_SMI_DISCARD);

	} else {

		/* C14-13:1 */
		if (hop_cnt && hop_ptr == hop_cnt + 1) {
			smp->hop_ptr--;
			return (smp->return_path[smp->hop_ptr] ==
				port_num ? IB_SMI_HANDLE : IB_SMI_DISCARD);
		}

		/* C14-13:2 */
		if (2 <= hop_ptr && hop_ptr <= hop_cnt) {
			if (node_type != RDMA_NODE_IB_SWITCH)
				return IB_SMI_DISCARD;

			/* smp->hop_ptr updated when sending */
			return (smp->return_path[hop_ptr-1] <= phys_port_cnt ?
				IB_SMI_HANDLE : IB_SMI_DISCARD);
		}

		/* C14-13:3 -- We're at the end of the DR segment of path */
		if (hop_ptr == 1) {
			if (smp->dr_slid == IB_LID_PERMISSIVE) {
				/* giving SMP to SM - update hop_ptr */
				smp->hop_ptr--;
				return IB_SMI_HANDLE;
			}
			/* smp->hop_ptr updated when sending */
			return (node_type == RDMA_NODE_IB_SWITCH ?
				IB_SMI_HANDLE : IB_SMI_DISCARD);
		}

		/* C14-13:4 -- hop_ptr = 0 -> give to SM */
		/* C14-13:5 -- Check for unreasonable hop pointer */
		return (hop_ptr == 0 ? IB_SMI_HANDLE : IB_SMI_DISCARD);
	}
}

enum smi_forward_action smi_check_forward_dr_smp(struct ib_smp *smp)
{
	u8 hop_ptr, hop_cnt;

	hop_ptr = smp->hop_ptr;
	hop_cnt = smp->hop_cnt;

	if (!ib_get_smp_direction(smp)) {
		/* C14-9:2 -- intermediate hop */
		if (hop_ptr && hop_ptr < hop_cnt)
			return IB_SMI_FORWARD;

		/* C14-9:3 -- at the end of the DR segment of path */
		if (hop_ptr == hop_cnt)
			return (smp->dr_dlid == IB_LID_PERMISSIVE ?
				IB_SMI_SEND : IB_SMI_LOCAL);

		/* C14-9:4 -- hop_ptr = hop_cnt + 1 -> give to SMA/SM */
		if (hop_ptr == hop_cnt + 1)
			return IB_SMI_SEND;
	} else {
		/* C14-13:2  -- intermediate hop */
		if (2 <= hop_ptr && hop_ptr <= hop_cnt)
			return IB_SMI_FORWARD;

		/* C14-13:3 -- at the end of the DR segment of path */
		if (hop_ptr == 1)
			return (smp->dr_slid != IB_LID_PERMISSIVE ?
				IB_SMI_SEND : IB_SMI_LOCAL);
	}
	return IB_SMI_LOCAL;
}

int smi_get_fwd_port(struct ib_smp *smp)
{
	return (!ib_get_smp_direction(smp) ? smp->initial_path[smp->hop_ptr+1] :
		smp->return_path[smp->hop_ptr-1]);
}
