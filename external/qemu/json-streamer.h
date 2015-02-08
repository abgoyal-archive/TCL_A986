

#ifndef QEMU_JSON_STREAMER_H
#define QEMU_JSON_STREAMER_H

#include "qlist.h"
#include "json-lexer.h"

typedef struct JSONMessageParser
{
    void (*emit)(struct JSONMessageParser *parser, QList *tokens);
    JSONLexer lexer;
    int brace_count;
    int bracket_count;
    QList *tokens;
} JSONMessageParser;

void json_message_parser_init(JSONMessageParser *parser,
                              void (*func)(JSONMessageParser *, QList *));

int json_message_parser_feed(JSONMessageParser *parser,
                             const char *buffer, size_t size);

int json_message_parser_flush(JSONMessageParser *parser);

void json_message_parser_destroy(JSONMessageParser *parser);

#endif
