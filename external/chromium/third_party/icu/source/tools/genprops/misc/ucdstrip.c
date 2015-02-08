
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* return the first character position after the end of the data */
static char *
endOfData(const char *l) {
    char *end;
    char c;

    end=strchr(l, '#');
    if(end!=NULL) {
        /* ignore whitespace before the comment */
        while(l!=end && ((c=*(end-1))==' ' || c=='\t')) {
            --end;
        }
    } else {
        end=strchr(l, 0);
    }
    return end;
}

extern int
main(int argc, const char *argv[]) {
    static char line[2000];
    char *end;

    while(gets(line)!=NULL) {
        if(strtol(line, &end, 16)>=0 && end!=line) {
            /* code point or range followed by semicolon and data, remove comment */
            *endOfData(line)=0;
        }
        puts(line);
    }

    return 0;
}
