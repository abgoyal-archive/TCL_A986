  
#include <linux/i2c.h>
#include <linux/delay.h>
#include <tpd.h>

#ifdef TPD_DEBUG_CODE
int tpd_fail_count = 0;
int tpd_trial_count = 0;

void tpd_debug_no_response(struct i2c_client *i2c_client) {
    char sleep[2] = {0x07,0x01};
    char wakeup[2] = {0x07,0x02};
    char threshold[2] = {0x09, 0x04};
    char gesture[2]   = {0x08, 0x11};
    char sleeptime[2]   = {0x0d, 0x01};
    char idletime[2]  = {0x0c, 0xff};
    static int trial_index = 0;
    static int delay_index = 0;
    int trial[] = {1,9,2,8,3,7,4,6,5};
    int delay[] = {1,100,2,50,3,25,4,12,5,6,6,1,1,500,500};
    int trial_max = 9;
    int delay_max = 15;
    int i,j;

    int wakeup_count = 200;

    for(i=0;i<trial[trial_index];i++) {
        i2c_master_send(i2c_client, sleep, 2);
        msleep(delay[delay_index]);
        for(j=0;j<wakeup_count;j++) {
            i2c_master_send(i2c_client, wakeup, 2);
            if(i2c_master_send(i2c_client, wakeup, 2)==2) break;
            msleep(10);
        }
        if(i2c_master_send(i2c_client,gesture,2)!=2) i=wakeup_count;
        if(i2c_master_send(i2c_client,threshold,2)!=2) i=wakeup_count;
        if(i2c_master_send(i2c_client,idletime,2)!=2) i=wakeup_count;
        if(i2c_master_send(i2c_client,sleeptime,2)!=2) i=wakeup_count;
        if(i==wakeup_count) tpd_fail_count++;
        tpd_trial_count++;
        printk("trial: %d    /    fail: %d\n", tpd_trial_count, tpd_fail_count);
        delay_index = ((delay_index+1)%delay_max);
    }
    trial_index = ((trial_index+1)%trial_max);
}

int tpd_debug_nr = 0;
module_param(tpd_debug_nr, int, 00644);
module_param(tpd_fail_count, int, 00644);
module_param(tpd_trial_count, int, 00644);

int  tpd_debug_time = 0;
long tpd_last_2_int_time[2] = {0};
long tpd_last_down_time = 0;
int  tpd_start_profiling = 0;

int tpd_down_status = 0;
module_param(tpd_debug_time, int, 00644);

void tpd_debug_set_time() {
    struct timeval t;

    if (!tpd_debug_time && !tpd_em_log) return;

    do_gettimeofday(&t);
    tpd_last_2_int_time[0] = tpd_last_2_int_time[1];
    tpd_last_2_int_time[1] = (t.tv_sec & 0xFFF) * 1000000 + t.tv_usec;

    //
    // Start profiling while receive touch DOWN event
    // Stop profiling while the first frame is upadted
    //
    if (!tpd_down_status) {
        tpd_start_profiling = 1;
        tpd_last_down_time = tpd_last_2_int_time[1];
    }
}
 
#ifdef TPD_DEBUG_TRACK
int DAL_Clean(void);
int DAL_Printf(const char *fmt, ...);
int LCD_LayerEnable(int id, BOOL enable);

int tpd_debug_track = 0;
int tpd_debug_track_color = 0;
int tpd_debug_touch_up = 0;
module_param(tpd_debug_track, int, 00644);

void tpd_draw(int x, int y) {
    UINT16 *buf = (UINT16 *)dal_fb_addr;
    buf = buf + (x+y*TPD_RES_X);
    tpd_debug_track_color += (tpd_debug_track_color>=31?0:1);
    *buf = (0xffe0+tpd_debug_track_color);
}

void tpd_down_debug_track(int x, int y) {
    if(tpd_debug_touch_up == 1) {
        DAL_Clean();
        tpd_debug_touch_up = 0;
    }
    LCD_LayerEnable(5, TRUE);
    tpd_draw(x-1, y-1); tpd_draw(x, y-1); tpd_draw(x+1, y-1);
    tpd_draw(x-1, y  );                   tpd_draw(x+1, y  );
    tpd_draw(x-1, y+1); tpd_draw(x, y+1); tpd_draw(x+1, y+1);
}

void tpd_up_debug_track(int x, int y) {
    if(x==0 && y==0) { tpd_debug_track_color = 0; DAL_Clean(); }
    tpd_debug_touch_up = 1;

}
#endif // TPD_DEBUG_TRACK


#define BUFFER_SIZE 128

int tpd_em_log = 0;
module_param(tpd_em_log, int, 00664);

int tpd_em_log_to_fs = 0;
module_param(tpd_em_log_to_fs, int, 00664);

int tpd_em_log_first = 1;

struct tpd_em_log_struct
{
    struct list_head list;
    char data[BUFFER_SIZE];
};
static LIST_HEAD(tpd_em_log_list);

static void *tpd_em_log_seq_start(struct seq_file *seq, loff_t *pos)
{
    struct tpd_em_log_struct *p;
    loff_t off = 0;

    list_for_each_entry(p, &tpd_em_log_list, list) {
        if (*pos == off++) return p;
    }
    return NULL;
}

static void *tpd_em_log_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
    struct list_head *n = ((struct tpd_em_log_struct *) v)->list.next;

    ++*pos;
    
    return (n != &tpd_em_log_list) ? list_entry(n, struct tpd_em_log_struct, list) : NULL;
}

static void tpd_em_log_seq_stop(struct seq_file *s, void *v) 
{ 
    struct tpd_em_log_struct *p, *p_tmp;
    list_for_each_entry_safe(p, p_tmp, &tpd_em_log_list, list) {
        list_del(&p->list);
        kfree(p);
    }
} 

static int tpd_em_log_seq_show(struct seq_file *seq, void *v) 
{
    const struct tpd_em_log_struct *p = v;
    seq_printf(seq, p->data);
    return 0;
} 

static struct seq_operations tpd_em_log_seq_ops = { 
    .start = tpd_em_log_seq_start,
    .next = tpd_em_log_seq_next,
    .stop = tpd_em_log_seq_stop,
    .show = tpd_em_log_seq_show 
};

static int tpd_em_log_open(struct inode *inode, struct file *file) { 
    return seq_open(file, &tpd_em_log_seq_ops);
};

static struct file_operations tpd_em_log_file_ops = { 
    .owner = THIS_MODULE,
    .open = tpd_em_log_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = seq_release 
};

void tpd_em_log_output(int raw_x, int raw_y, int cal_x, int cal_y, int p, int down) {
    if (down == TPD_TYPE_INT_DOWN) {
        printk("[tpd_em_log] int trigger down\n");
    } else if (down == TPD_TYPE_INT_UP) {
        printk("[tpd_em_log] int trigger up\n");
    } else if (down == TPD_TYPE_TIMER) {
        printk("[tpd_em_log] timer trigger\n");
    } else if (down == TPD_TYPE_RAW_DATA) {
        if (tpd_em_log == TPD_TYPE_RAW_DATA) {
            printk("[tpd_em_log] rx=%d,ry=%d,rz1=%d,rz2=%d,p=%d,r\n",
                raw_x, raw_y, cal_x, cal_y, p);
        }
    } else if(down == TPD_TYPE_REJECT1){
    		printk("[tpd_em_log] the first or last point is rejected\n");
    }else if(down == TPD_TYPE_REJECT2){
    		printk("[tpd_em_log] pressure(%d) > NICE_PRESSURE(%d)\n", raw_x, raw_y);
    } else if(down == TPD_TYPE_FIST_LATENCY) {
    		printk("[tpd_em_log] The first touch latency is %d ms\n", raw_x/1000);
    } else if (down && tpd_down_status == 0) {
        printk("[tpd_em_log] rx=%d,ry=%d,cx=%d,cy=%d,p=%d,d(+%ld ms)\n",
            raw_x, raw_y, cal_x, cal_y, p, (tpd_last_2_int_time[1] - tpd_last_2_int_time[0]) / 1000);
    } else if (down && tpd_down_status != 0) {
        printk("[tpd_em_log] rx=%d,ry=%d,cx=%d,cy=%d,p=%d,d(+%ld ms)\n",
            raw_x, raw_y, cal_x, cal_y, p, (tpd_last_2_int_time[1] - tpd_last_2_int_time[0]) / 1000);
    } else {
        printk("[tpd_em_log] rx=%d,ry=%d,cx=%d,cy=%d,p=%d,u(+%ld ms)\n",
            raw_x, raw_y, cal_x, cal_y, p, (tpd_last_2_int_time[1] - tpd_last_2_int_time[0]) / 1000);
    }
}
                
void tpd_em_log_store(int raw_x, int raw_y, int cal_x, int cal_y, int p, int down) {
    static struct proc_dir_entry *entry = NULL;
    struct tpd_em_log_struct *tpd_em_log_struct_new;
    struct timeval t;
    
    if (tpd_em_log_first) {
        entry = create_proc_entry("tpd_em_log", 0, NULL);
        if (entry) { 
            entry->proc_fops = &tpd_em_log_file_ops;
        } 
        tpd_em_log_first = 0;
        return;
    }
    
    tpd_em_log_struct_new = kmalloc(sizeof(struct tpd_em_log_struct), GFP_ATOMIC);

    do_gettimeofday(&t);
    
    if (down == TPD_TYPE_INT_DOWN) {
        snprintf(tpd_em_log_struct_new->data, BUFFER_SIZE, "[%5lu.%06lu][tpd_em_log] int trigger down\n", (t.tv_sec & 0xFFF), t.tv_usec);
    } else if (down == TPD_TYPE_INT_UP) {
        snprintf(tpd_em_log_struct_new->data, BUFFER_SIZE, "[%5lu.%06lu][tpd_em_log] int trigger up\n", (t.tv_sec & 0xFFF), t.tv_usec);
    } else if (down == TPD_TYPE_TIMER) {
        snprintf(tpd_em_log_struct_new->data, BUFFER_SIZE, "[%5lu.%06lu][tpd_em_log] timer trigger\n", (t.tv_sec & 0xFFF), t.tv_usec);
    } else if (down == TPD_TYPE_RAW_DATA) {
        if (tpd_em_log == TPD_TYPE_RAW_DATA) {
            snprintf(tpd_em_log_struct_new->data, BUFFER_SIZE, "[%5lu.%06lu][tpd_em_log] rx=%d,ry=%d,rz1=%d,rz2=%d,p=%d,r\n", 
                    (t.tv_sec & 0xFFF), t.tv_usec, raw_x, raw_y, cal_x, cal_y, p);
        }
    } else if(down == TPD_TYPE_REJECT1){
    		snprintf(tpd_em_log_struct_new->data, BUFFER_SIZE, "[%5lu.%06lu][tpd_em_log] the first or last point is rejected\n", (t.tv_sec & 0xFFF),  t.tv_usec);
    }else if(down == TPD_TYPE_REJECT2){
    		snprintf(tpd_em_log_struct_new->data, BUFFER_SIZE, "[%5lu.%06lu][tpd_em_log] pressure(%d) > NICE_PRESSURE(%d)\n",
    						(t.tv_sec & 0xFFF), t.tv_usec, raw_x, raw_y);
    } else if(down == TPD_TYPE_FIST_LATENCY) {
    	snprintf(tpd_em_log_struct_new->data, BUFFER_SIZE,"[%5lu.%06lu][tpd_em_log] The first touch latency is %d ms\n", (t.tv_sec & 0xFFF), t.tv_usec, raw_x/1000);
    } else if (down && tpd_down_status == 0) {
        snprintf(tpd_em_log_struct_new->data, BUFFER_SIZE, "[%5lu.%06lu][tpd_em_log] rx=%d,ry=%d,cx=%d,cy=%d,p=%d,d(+%ld ms)\n", 
                (t.tv_sec & 0xFFF), t.tv_usec, raw_x, raw_y, cal_x, cal_y, p, (tpd_last_2_int_time[1] - tpd_last_2_int_time[0]) / 1000);
    } else if (down && tpd_down_status != 0) {
        snprintf(tpd_em_log_struct_new->data, BUFFER_SIZE, "[%5lu.%06lu][tpd_em_log] rx=%d,ry=%d,cx=%d,cy=%d,p=%d,d(+%ld ms)\n", 
                (t.tv_sec & 0xFFF), t.tv_usec, raw_x, raw_y, cal_x, cal_y, p, (tpd_last_2_int_time[1] - tpd_last_2_int_time[0]) / 1000);
    } else {
        snprintf(tpd_em_log_struct_new->data, BUFFER_SIZE, "[%5lu.%06lu][tpd_em_log] rx=%d,ry=%d,cx=%d,cy=%d,p=%d,u(+%ld ms)\n", 
                (t.tv_sec & 0xFFF), t.tv_usec, raw_x, raw_y, cal_x, cal_y, p, (tpd_last_2_int_time[1] - tpd_last_2_int_time[0]) / 1000);
    }
      
    list_add_tail(&tpd_em_log_struct_new->list, &tpd_em_log_list);
}

void tpd_em_log_release(void) {
    struct tpd_em_log_struct *p, *p_tmp;
    if (!tpd_em_log_first) {
        remove_proc_entry("tpd_em_log", NULL);
   
        list_for_each_entry_safe(p, p_tmp, &tpd_em_log_list, list) {
            list_del(&p->list);
            kfree(p);
        }
    
        tpd_em_log_first = 1;
        tpd_em_log_to_fs = 0;
    }
}

int tpd_em_debuglog = 0;
module_param(tpd_em_debuglog, int, 00664);

#endif // TPD_DEBUG_CODE
