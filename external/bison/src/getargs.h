
#ifndef GETARGS_H_
# define GETARGS_H_

/* flags set by % directives */

/* for -S */
extern char const *skeleton;

/* for -I */
extern char const *include;

extern bool debug_flag;			/* for -t */
extern bool defines_flag;		/* for -d */
extern bool graph_flag;			/* for -g */
extern bool locations_flag;
extern bool no_lines_flag;		/* for -l */
extern bool no_parser_flag;		/* for -n */
extern bool token_table_flag;		/* for -k */
extern bool yacc_flag;			/* for -y */

extern bool error_verbose;



extern bool glr_parser;


extern bool pure_parser;


extern bool nondeterministic_parser;

/* --trace.  */
enum trace
  {
    trace_none      = 0,
    trace_scan      = 1 << 0,
    trace_parse     = 1 << 1,
    trace_resource  = 1 << 2,
    trace_sets      = 1 << 3,
    trace_bitsets   = 1 << 4,
    trace_tools     = 1 << 5,
    trace_automaton = 1 << 6,
    trace_grammar   = 1 << 7,
    trace_time      = 1 << 8,
    trace_skeleton  = 1 << 9,
    trace_m4        = 1 << 10,
    trace_all       = ~0
  };
extern int trace_flag;

/* --report.  */
enum report
  {
    report_none             = 0,
    report_states           = 1 << 0,
    report_itemsets         = 1 << 1,
    report_look_ahead_tokens= 1 << 2,
    report_solved_conflicts = 1 << 3,
    report_all              = ~0
  };
extern int report_flag;

void getargs (int argc, char *argv[]);

#endif /* !GETARGS_H_ */
