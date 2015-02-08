

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>

#include <pcmcia/cs_types.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/ss.h>
#include <pcmcia/cs.h>
#include <pcmcia/ds.h>
#include "cs_internal.h"


int pccard_read_tuple(struct pcmcia_socket *s, unsigned int function,
		cisdata_t code, void *parse)
{
	tuple_t tuple;
	cisdata_t *buf;
	int ret;

	buf = kmalloc(256, GFP_KERNEL);
	if (buf == NULL) {
		dev_printk(KERN_WARNING, &s->dev, "no memory to read tuple\n");
		return -ENOMEM;
	}
	tuple.DesiredTuple = code;
	tuple.Attributes = 0;
	if (function == BIND_FN_ALL)
		tuple.Attributes = TUPLE_RETURN_COMMON;
	ret = pccard_get_first_tuple(s, function, &tuple);
	if (ret != 0)
		goto done;
	tuple.TupleData = buf;
	tuple.TupleOffset = 0;
	tuple.TupleDataMax = 255;
	ret = pccard_get_tuple_data(s, &tuple);
	if (ret != 0)
		goto done;
	ret = pcmcia_parse_tuple(&tuple, parse);
done:
	kfree(buf);
	return ret;
}


int pccard_loop_tuple(struct pcmcia_socket *s, unsigned int function,
		      cisdata_t code, cisparse_t *parse, void *priv_data,
		      int (*loop_tuple) (tuple_t *tuple,
					 cisparse_t *parse,
					 void *priv_data))
{
	tuple_t tuple;
	cisdata_t *buf;
	int ret;

	buf = kzalloc(256, GFP_KERNEL);
	if (buf == NULL) {
		dev_printk(KERN_WARNING, &s->dev, "no memory to read tuple\n");
		return -ENOMEM;
	}

	tuple.TupleData = buf;
	tuple.TupleDataMax = 255;
	tuple.TupleOffset = 0;
	tuple.DesiredTuple = code;
	tuple.Attributes = 0;

	ret = pccard_get_first_tuple(s, function, &tuple);
	while (!ret) {
		if (pccard_get_tuple_data(s, &tuple))
			goto next_entry;

		if (parse)
			if (pcmcia_parse_tuple(&tuple, parse))
				goto next_entry;

		ret = loop_tuple(&tuple, parse, priv_data);
		if (!ret)
			break;

next_entry:
		ret = pccard_get_next_tuple(s, function, &tuple);
	}

	kfree(buf);
	return ret;
}

struct pcmcia_cfg_mem {
	struct pcmcia_device *p_dev;
	void *priv_data;
	int (*conf_check) (struct pcmcia_device *p_dev,
			   cistpl_cftable_entry_t *cfg,
			   cistpl_cftable_entry_t *dflt,
			   unsigned int vcc,
			   void *priv_data);
	cisparse_t parse;
	cistpl_cftable_entry_t dflt;
};

static int pcmcia_do_loop_config(tuple_t *tuple, cisparse_t *parse, void *priv)
{
	cistpl_cftable_entry_t *cfg = &parse->cftable_entry;
	struct pcmcia_cfg_mem *cfg_mem = priv;

	/* default values */
	cfg_mem->p_dev->conf.ConfigIndex = cfg->index;
	if (cfg->flags & CISTPL_CFTABLE_DEFAULT)
		cfg_mem->dflt = *cfg;

	return cfg_mem->conf_check(cfg_mem->p_dev, cfg, &cfg_mem->dflt,
				   cfg_mem->p_dev->socket->socket.Vcc,
				   cfg_mem->priv_data);
}

int pcmcia_loop_config(struct pcmcia_device *p_dev,
		       int	(*conf_check)	(struct pcmcia_device *p_dev,
						 cistpl_cftable_entry_t *cfg,
						 cistpl_cftable_entry_t *dflt,
						 unsigned int vcc,
						 void *priv_data),
		       void *priv_data)
{
	struct pcmcia_cfg_mem *cfg_mem;
	int ret;

	cfg_mem = kzalloc(sizeof(struct pcmcia_cfg_mem), GFP_KERNEL);
	if (cfg_mem == NULL)
		return -ENOMEM;

	cfg_mem->p_dev = p_dev;
	cfg_mem->conf_check = conf_check;
	cfg_mem->priv_data = priv_data;

	ret = pccard_loop_tuple(p_dev->socket, p_dev->func,
				CISTPL_CFTABLE_ENTRY, &cfg_mem->parse,
				cfg_mem, pcmcia_do_loop_config);

	kfree(cfg_mem);
	return ret;
}
EXPORT_SYMBOL(pcmcia_loop_config);


struct pcmcia_loop_mem {
	struct pcmcia_device *p_dev;
	void *priv_data;
	int (*loop_tuple) (struct pcmcia_device *p_dev,
			   tuple_t *tuple,
			   void *priv_data);
};

static int pcmcia_do_loop_tuple(tuple_t *tuple, cisparse_t *parse, void *priv)
{
	struct pcmcia_loop_mem *loop = priv;

	return loop->loop_tuple(loop->p_dev, tuple, loop->priv_data);
};

int pcmcia_loop_tuple(struct pcmcia_device *p_dev, cisdata_t code,
		      int (*loop_tuple) (struct pcmcia_device *p_dev,
					 tuple_t *tuple,
					 void *priv_data),
		      void *priv_data)
{
	struct pcmcia_loop_mem loop = {
		.p_dev = p_dev,
		.loop_tuple = loop_tuple,
		.priv_data = priv_data};

	return pccard_loop_tuple(p_dev->socket, p_dev->func, code, NULL,
				 &loop, pcmcia_do_loop_tuple);
}
EXPORT_SYMBOL(pcmcia_loop_tuple);


struct pcmcia_loop_get {
	size_t len;
	cisdata_t **buf;
};

static int pcmcia_do_get_tuple(struct pcmcia_device *p_dev, tuple_t *tuple,
			       void *priv)
{
	struct pcmcia_loop_get *get = priv;

	*get->buf = kzalloc(tuple->TupleDataLen, GFP_KERNEL);
	if (*get->buf) {
		get->len = tuple->TupleDataLen;
		memcpy(*get->buf, tuple->TupleData, tuple->TupleDataLen);
	} else
		dev_dbg(&p_dev->dev, "do_get_tuple: out of memory\n");
	return 0;
}

size_t pcmcia_get_tuple(struct pcmcia_device *p_dev, cisdata_t code,
			unsigned char **buf)
{
	struct pcmcia_loop_get get = {
		.len = 0,
		.buf = buf,
	};

	*get.buf = NULL;
	pcmcia_loop_tuple(p_dev, code, pcmcia_do_get_tuple, &get);

	return get.len;
}
EXPORT_SYMBOL(pcmcia_get_tuple);


static int pcmcia_do_get_mac(struct pcmcia_device *p_dev, tuple_t *tuple,
			     void *priv)
{
	struct net_device *dev = priv;
	int i;

	if (tuple->TupleData[0] != CISTPL_FUNCE_LAN_NODE_ID)
		return -EINVAL;
	if (tuple->TupleDataLen < ETH_ALEN + 2) {
		dev_warn(&p_dev->dev, "Invalid CIS tuple length for "
			"LAN_NODE_ID\n");
		return -EINVAL;
	}

	if (tuple->TupleData[1] != ETH_ALEN) {
		dev_warn(&p_dev->dev, "Invalid header for LAN_NODE_ID\n");
		return -EINVAL;
	}
	for (i = 0; i < 6; i++)
		dev->dev_addr[i] = tuple->TupleData[i+2];
	return 0;
}

int pcmcia_get_mac_from_cis(struct pcmcia_device *p_dev, struct net_device *dev)
{
	return pcmcia_loop_tuple(p_dev, CISTPL_FUNCE, pcmcia_do_get_mac, dev);
}
EXPORT_SYMBOL(pcmcia_get_mac_from_cis);

