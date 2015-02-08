
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <asm/uaccess.h>
#include "prof_ctl.h"
#include "prof_mem.h"

#include <linux/pid.h>
#define SEQ_printf(m, x...)	    \
 do {			    \
    if (m)		    \
	seq_printf(m, x);	\
    else		    \
	printk(x);	    \
 } while (0)

#define MT_DEBUG_ENTRY(name) \
static int mt_##name##_show(struct seq_file *m, void *v);\
static int mt_##name##_write(struct file *filp, const char *ubuf, size_t cnt, loff_t *data);\
static int mt_##name##_open(struct inode *inode, struct file *file) \
{ \
    return single_open(file, mt_##name##_show, inode->i_private); \
} \
\
static const struct file_operations mt_##name##_fops = { \
    .open = mt_##name##_open, \
    .write = mt_##name##_write,\
    .read = seq_read, \
    .llseek = seq_lseek, \
    .release = single_release, \
};\
void mt_##name##_switch(int on);

static long long nsec_high(unsigned long long nsec)
{
    if ((long long)nsec < 0) {
	nsec = -nsec;
	do_div(nsec, 1000000);
	return -nsec;
    }
    do_div(nsec, 1000000);

    return nsec;
}

static unsigned long nsec_low(unsigned long long nsec)
{
    if ((long long)nsec < 0)
	nsec = -nsec;

    return do_div(nsec, 1000000);
}
#define SPLIT_NS(x) nsec_high(x), nsec_low(x)

static long long usec_high(unsigned long long usec)
{
    if ((long long)usec < 0) {
	usec = -usec;
	do_div(usec, 1000);
	return -usec;
    }
    do_div(usec, 1000);

    return usec;
}

static unsigned long usec_low(unsigned long long usec)
{
    if ((long long)usec < 0)
	usec = -usec;

    return do_div(usec, 1000);
}

#define SPLIT_US(x) usec_high(x), usec_low(x)

static void print_task(struct seq_file *m, struct task_struct *p)
{
    SEQ_printf(m, "%15s %5d %9Ld %5d ",
	p->comm, p->pid,
	(long long)(p->nvcsw + p->nivcsw),
	p->prio);
#ifdef CONFIG_SCHEDSTATS
    SEQ_printf(m, "%9Ld.%06ld %9Ld.%06ld %9Ld.%06ld\n",
	SPLIT_NS(p->se.vruntime),
	SPLIT_NS(p->se.sum_exec_runtime),
	SPLIT_NS(p->se.statistics.sum_sleep_runtime));
#else
    SEQ_printf(m, "%15Ld %15Ld %15Ld.%06ld %15Ld.%06ld %15Ld.%06ld\n",
	0LL, 0LL, 0LL, 0L, 0LL, 0L, 0LL, 0L);
#endif
}
/*========================================================================*/
/* Real work */
/*========================================================================*/
/* 1. sched info */
MT_DEBUG_ENTRY(sched_debug);
static int mt_sched_debug_show(struct seq_file *m, void *v)
{
    struct task_struct *g, *p;
    unsigned long flags;
    SEQ_printf(m, "=== mt Scheduler Profiling ===\n");
    SEQ_printf(m,
	    "\nrunnable tasks:\n"
	    "            task   PID   switches  prio"
	    "     exec-runtime         sum-exec        sum-sleep\n"
	    "------------------------------------------------------"
	    "----------------------------------------------------\n");
    read_lock_irqsave(&tasklist_lock, flags);

    do_each_thread(g, p) {
	print_task(m, p);
    } while_each_thread(g, p);

    read_unlock_irqrestore(&tasklist_lock, flags);
    return 0;
}
static ssize_t mt_sched_debug_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
    return cnt;
}

/* 2. cputime */
MT_DEBUG_ENTRY(cputime);
static int mt_cputime_show(struct seq_file *m, void *v)
{

#ifdef CONFIG_MTPROF_CPUTIME
    int i;
    unsigned long long end_ts;
    struct task_struct *tsk;
    char status;
    if(0 == prof_end_ts)
	end_ts = sched_clock();
    else
	end_ts = prof_end_ts; 

    SEQ_printf(m,"iowait time: %llu\n",
	cpu0_iowait_end == 0 ? mtprof_get_cpu_iowait(0) - cpu0_iowait_start:
			cpu0_iowait_end - cpu0_iowait_start
    );

    SEQ_printf(m, "-----------------------------------------------\n");
    SEQ_printf(m, "        Duration: %10Ld.%06ld ms \n", SPLIT_NS(end_ts - prof_start_ts));
    SEQ_printf(m, "        --------------------------------\n");
    SEQ_printf(m, "           Start: %10Ld.%06ld ms \n", SPLIT_NS(prof_start_ts));
    SEQ_printf(m, "             End: %10Ld.%06ld ms \n", SPLIT_NS(end_ts));
    SEQ_printf(m, "-----------------------------------------------\n");
    SEQ_printf(m, "         Process:Status: PID:TGID:          CPUtime:         Elapsed:   User: Kernel\n");
    SEQ_printf(m, "            Idle:     L:   0:   0:%10Ld.%06ld:%10Ld.%06ld\n",
	SPLIT_US(cpu0_idletime_end == 0 ? mtprof_get_cpu_idle(0) - cpu0_idletime_start:
			cpu0_idletime_end - cpu0_idletime_start),
	SPLIT_NS(end_ts - prof_start_ts)
    );
    for(i=0;i<proc_count; i++){
	/* Record new cputime*/
	tsk = find_task_by_vpid(mt_proc[i]->pid);

	if(tsk!=NULL){
	    /* update cputime */
	    if(mtsched_enabled){
		mt_proc[i]->cputime = tsk->se.sum_exec_runtime;
		mt_task_times(tsk, &mt_proc[i]->utime, &mt_proc[i]->stime);
		mt_proc[i]->utime = cputime_sub(mt_proc[i]->utime, mt_proc[i]->utime_init);
		mt_proc[i]->stime = cputime_sub(mt_proc[i]->stime, mt_proc[i]->stime_init);
	    }
	    status = 'L';    
	}
	else{
	    status = 'D';    
	}

	SEQ_printf(m,"%16s:     %c:%4d:%4d:%10Ld.%06ld:%10Ld.%06ld:%7u:%7u\n", 
	    mt_proc[i]->comm,
	    status,
	    mt_proc[i]->pid,
	    mt_proc[i]->tgid,
	    SPLIT_NS(mt_proc[i]->cputime - mt_proc[i]->cputime_init),
	    SPLIT_NS(mt_proc[i]->prof_end== 0? end_ts - mt_proc[i]->prof_start: mt_proc[i]->prof_end -  mt_proc[i]->prof_start),
	    cputime_to_msecs(mt_proc[i]->utime),
	    cputime_to_msecs(mt_proc[i]->stime)
	);
    }
#endif
    return 0;
}

static ssize_t mt_cputime_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
#ifdef CONFIG_MTPROF_CPUTIME
    char buf[64];
    unsigned long val;
    int ret;

    if (cnt >= sizeof(buf))
	return -EINVAL;

    if (copy_from_user(&buf, ubuf, cnt))
	return -EFAULT;

    buf[cnt] = 0;

    ret = strict_strtoul(buf, 10, &val);
    if (ret < 0)
	return ret;
    printk("mtsched_proc input stream:%s\n", buf);
    val = !!val;
    //0: off, 1:on
    mt_cputime_switch(val);
#endif
    return cnt;

}

/* 3. mem prof*/
MT_DEBUG_ENTRY(memprof);
static int mt_memprof_show(struct seq_file *m, void *v)
{
	unsigned long long avg_good_ns = good_pages_time;
	unsigned long long avg_bad_ns = bad_pages_time;

	SEQ_printf(m, "----------------------------------------\n");
	SEQ_printf(m, "Total Mem AllocPage Latency(ms): %Ld.%06ld \n", SPLIT_NS(good_pages_time + bad_pages_time));
	SEQ_printf(m, "----------------------------------------\n");

	if (num_good_pages) {
		SEQ_printf(m, "Alloc Good Pages Latency(ms): %Ld.%06lu \n", SPLIT_NS(good_pages_time));
		do_div(avg_good_ns, num_good_pages);
		SEQ_printf(m, "Good pages: %lu\n", num_good_pages);
		SEQ_printf(m, "Avg. Good Page Latency(ns): %llu\n", avg_good_ns);
	}
	if (num_bad_pages) {
		SEQ_printf(m, "Alloc Bad Pages Latency(ms): %Ld.%06lu \n", SPLIT_NS(bad_pages_time));
		do_div(avg_bad_ns, num_bad_pages);
		SEQ_printf(m, "Bad pages: %lu\n", num_bad_pages);
		SEQ_printf(m, "Avg. Bad Page Latency(ns): %llu\n", avg_bad_ns);
	}

    return 0; 
}

static ssize_t mt_memprof_write(struct file *filp, const char *ubuf,
	size_t cnt, loff_t *data)
{
    char buf[64];
    unsigned long val;
    int ret;

    if (cnt >= sizeof(buf))
	return -EINVAL;

    if (copy_from_user(&buf, ubuf, cnt))
	return -EFAULT;

    buf[cnt] = 0;

    ret = strict_strtoul(buf, 10, &val);
    if (ret < 0)
	return ret;
    printk("allocpages input stream:%s\n", buf);
    val = !!val;
    //0: off, 1:on
    mt_memprof_switch(val);

    return cnt;
}
/* 4. prof status*/
MT_DEBUG_ENTRY(status);
#define MT_CPUTIME 1
#define MT_MEMPROF 2
unsigned long mtprof_status = 0;
static int mt_status_show(struct seq_file *m, void *v)
{
    SEQ_printf(m, "%lu\n", mtprof_status);
    return 0; 
}

static ssize_t mt_status_write(struct file *filp, const char *ubuf,
	size_t cnt, loff_t *data)
{
    char buf[64];
    unsigned long val;
    int ret;
    if (cnt >= sizeof(buf))
	return -EINVAL;

    if (copy_from_user(&buf, ubuf, cnt))
	return -EFAULT;

    buf[cnt] = 0;

    ret = strict_strtoul(buf, 10, &val);
    if (ret < 0)
	return ret;
    //0: off, 1:on
    printk("[mtprof] status = 0x%x\n", (unsigned int)mtprof_status);
    mtprof_status = val;
    printk("[mtprof] new status = 0x%x\n", (unsigned int)mtprof_status);
    return cnt;
}
/*-------------------------------------------------------------------*/
static int __init init_mtsched_prof(void)
{
    struct proc_dir_entry *pe;
    if (!proc_mkdir("mtprof", NULL)){
	return -1;
    }
    pe = proc_create("mtprof/sched", 0444, NULL, &mt_sched_debug_fops);
    if (!pe)
	return -ENOMEM;
    pe = proc_create("mtprof/cputime", 0666, NULL, &mt_cputime_fops);
    if (!pe)
	return -ENOMEM;
    pe = proc_create("mtprof/memprof", 0666, NULL, &mt_memprof_fops);
    if (!pe)
	return -ENOMEM;
    pe = proc_create("mtprof/status", 0666, NULL, &mt_status_fops);
    if (!pe)
	return -ENOMEM;
    //start_record_task();
    return 0;
}
__initcall(init_mtsched_prof);
