
#ifndef GCC_TIMEVAR_H
#define GCC_TIMEVAR_H



struct timevar_time_def
{
  /* User time in this process.  */
  float user;

  /* System time (if applicable for this host platform) in this
     process.  */
  float sys;

  /* Wall clock time.  */
  float wall;
};


#define DEFTIMEVAR(identifier__, name__) \
    identifier__,
typedef enum
{
#include "timevar.def"
  TIMEVAR_LAST
}
timevar_id_t;
#undef DEFTIMEVAR

extern void init_timevar (void);
extern void timevar_push (timevar_id_t);
extern void timevar_pop (timevar_id_t);
extern void timevar_start (timevar_id_t);
extern void timevar_stop (timevar_id_t);
extern void timevar_get (timevar_id_t, struct timevar_time_def *);
extern void timevar_print (FILE *);

/* Provided for backward compatibility.  */
extern long get_run_time (void);
extern void print_time (const char *, long);

extern int timevar_report;

#endif /* ! GCC_TIMEVAR_H */
