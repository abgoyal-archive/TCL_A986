
#include <linux/percpu.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>

#include "rds.h"
#include "tcp.h"

DEFINE_PER_CPU(struct rds_tcp_statistics, rds_tcp_stats)
	____cacheline_aligned;

static const char const *rds_tcp_stat_names[] = {
	"tcp_data_ready_calls",
	"tcp_write_space_calls",
	"tcp_sndbuf_full",
	"tcp_connect_raced",
	"tcp_listen_closed_stale",
};

unsigned int rds_tcp_stats_info_copy(struct rds_info_iterator *iter,
				     unsigned int avail)
{
	struct rds_tcp_statistics stats = {0, };
	uint64_t *src;
	uint64_t *sum;
	size_t i;
	int cpu;

	if (avail < ARRAY_SIZE(rds_tcp_stat_names))
		goto out;

	for_each_online_cpu(cpu) {
		src = (uint64_t *)&(per_cpu(rds_tcp_stats, cpu));
		sum = (uint64_t *)&stats;
		for (i = 0; i < sizeof(stats) / sizeof(uint64_t); i++)
			*(sum++) += *(src++);
	}

	rds_stats_info_copy(iter, (uint64_t *)&stats, rds_tcp_stat_names,
			    ARRAY_SIZE(rds_tcp_stat_names));
out:
	return ARRAY_SIZE(rds_tcp_stat_names);
}
