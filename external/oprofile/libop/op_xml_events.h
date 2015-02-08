
#ifndef OP_XML_EVENTS_H
#define OP_XML_EVENTS_H

#include "op_events.h"

void xml_help_for_event(struct op_event const * event);
void open_xml_events(char const * title, char const * doc, op_cpu cpu_type);
void close_xml_events(void);

#endif /* OP_XML_EVENTS_H */
