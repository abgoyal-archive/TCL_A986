
extern int lineno;

struct keyword_t {
	char *string;
	int type;
};

extern struct keyword_t rfcomm_keyword[];

int rfcomm_find_keyword(struct keyword_t *keyword, char *string);

#define MAXCOMMENTLEN  100

struct rfcomm_opts {
	int bind;
	bdaddr_t bdaddr;
	int channel;
	char comment[MAXCOMMENTLEN + 1];
};

extern struct rfcomm_opts rfcomm_opts[RFCOMM_MAX_DEV];

int rfcomm_read_config(char *filename);
