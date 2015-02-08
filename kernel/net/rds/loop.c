
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/in.h>

#include "rds.h"
#include "loop.h"

static DEFINE_SPINLOCK(loop_conns_lock);
static LIST_HEAD(loop_conns);


static int rds_loop_xmit(struct rds_connection *conn, struct rds_message *rm,
			 unsigned int hdr_off, unsigned int sg,
			 unsigned int off)
{
	BUG_ON(hdr_off || sg || off);

	rds_inc_init(&rm->m_inc, conn, conn->c_laddr);
	rds_message_addref(rm); /* for the inc */

	rds_recv_incoming(conn, conn->c_laddr, conn->c_faddr, &rm->m_inc,
			  GFP_KERNEL, KM_USER0);

	rds_send_drop_acked(conn, be64_to_cpu(rm->m_inc.i_hdr.h_sequence),
			    NULL);

	rds_inc_put(&rm->m_inc);

	return sizeof(struct rds_header) + be32_to_cpu(rm->m_inc.i_hdr.h_len);
}

static int rds_loop_xmit_cong_map(struct rds_connection *conn,
				  struct rds_cong_map *map,
				  unsigned long offset)
{
	BUG_ON(offset);
	BUG_ON(map != conn->c_lcong);

	rds_cong_map_updated(conn->c_fcong, ~(u64) 0);

	return sizeof(struct rds_header) + RDS_CONG_MAP_BYTES;
}

/* we need to at least give the thread something to succeed */
static int rds_loop_recv(struct rds_connection *conn)
{
	return 0;
}

struct rds_loop_connection {
	struct list_head loop_node;
	struct rds_connection *conn;
};

static int rds_loop_conn_alloc(struct rds_connection *conn, gfp_t gfp)
{
	struct rds_loop_connection *lc;
	unsigned long flags;

	lc = kzalloc(sizeof(struct rds_loop_connection), GFP_KERNEL);
	if (lc == NULL)
		return -ENOMEM;

	INIT_LIST_HEAD(&lc->loop_node);
	lc->conn = conn;
	conn->c_transport_data = lc;

	spin_lock_irqsave(&loop_conns_lock, flags);
	list_add_tail(&lc->loop_node, &loop_conns);
	spin_unlock_irqrestore(&loop_conns_lock, flags);

	return 0;
}

static void rds_loop_conn_free(void *arg)
{
	struct rds_loop_connection *lc = arg;
	rdsdebug("lc %p\n", lc);
	list_del(&lc->loop_node);
	kfree(lc);
}

static int rds_loop_conn_connect(struct rds_connection *conn)
{
	rds_connect_complete(conn);
	return 0;
}

static void rds_loop_conn_shutdown(struct rds_connection *conn)
{
}

void rds_loop_exit(void)
{
	struct rds_loop_connection *lc, *_lc;
	LIST_HEAD(tmp_list);

	/* avoid calling conn_destroy with irqs off */
	spin_lock_irq(&loop_conns_lock);
	list_splice(&loop_conns, &tmp_list);
	INIT_LIST_HEAD(&loop_conns);
	spin_unlock_irq(&loop_conns_lock);

	list_for_each_entry_safe(lc, _lc, &tmp_list, loop_node) {
		WARN_ON(lc->conn->c_passive);
		rds_conn_destroy(lc->conn);
	}
}

struct rds_transport rds_loop_transport = {
	.xmit			= rds_loop_xmit,
	.xmit_cong_map		= rds_loop_xmit_cong_map,
	.recv			= rds_loop_recv,
	.conn_alloc		= rds_loop_conn_alloc,
	.conn_free		= rds_loop_conn_free,
	.conn_connect		= rds_loop_conn_connect,
	.conn_shutdown		= rds_loop_conn_shutdown,
	.inc_copy_to_user	= rds_message_inc_copy_to_user,
	.inc_purge		= rds_message_inc_purge,
	.inc_free		= rds_message_inc_free,
	.t_name			= "loopback",
};
