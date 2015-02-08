
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef BUS_CONFIG_PARSER_H
#define BUS_CONFIG_PARSER_H

#include <config.h>

#include <dbus/dbus.h>
#include <dbus/dbus-string.h>
#include <dbus/dbus-list.h>
#include <dbus/dbus-hash.h>
#include "bus.h"

/* Whatever XML library we're using just pushes data into this API */

typedef struct BusConfigParser BusConfigParser;

BusConfigParser* bus_config_parser_new (const DBusString      *basedir,
                                        dbus_bool_t            is_toplevel,
                                        const BusConfigParser *parent);

BusConfigParser* bus_config_parser_ref           (BusConfigParser   *parser);
void             bus_config_parser_unref         (BusConfigParser   *parser);
dbus_bool_t      bus_config_parser_check_doctype (BusConfigParser   *parser,
                                                  const char        *doctype,
                                                  DBusError         *error);
dbus_bool_t      bus_config_parser_start_element (BusConfigParser   *parser,
                                                  const char        *element_name,
                                                  const char       **attribute_names,
                                                  const char       **attribute_values,
                                                  DBusError         *error);
dbus_bool_t      bus_config_parser_end_element   (BusConfigParser   *parser,
                                                  const char        *element_name,
                                                  DBusError         *error);
dbus_bool_t      bus_config_parser_content       (BusConfigParser   *parser,
                                                  const DBusString  *content,
                                                  DBusError         *error);
dbus_bool_t      bus_config_parser_finished      (BusConfigParser   *parser,
                                                  DBusError         *error);

/* Functions for extracting the parse results */
const char* bus_config_parser_get_user         (BusConfigParser *parser);
const char* bus_config_parser_get_type         (BusConfigParser *parser);
DBusList**  bus_config_parser_get_addresses    (BusConfigParser *parser);
DBusList**  bus_config_parser_get_mechanisms   (BusConfigParser *parser);
dbus_bool_t bus_config_parser_get_fork         (BusConfigParser *parser);
const char* bus_config_parser_get_pidfile      (BusConfigParser *parser);
DBusList**  bus_config_parser_get_service_dirs (BusConfigParser *parser);
DBusList**  bus_config_parser_get_conf_dirs    (BusConfigParser *parser);
BusPolicy*  bus_config_parser_steal_policy     (BusConfigParser *parser);
void        bus_config_parser_get_limits       (BusConfigParser *parser,
                                                BusLimits       *limits);

DBusHashTable* bus_config_parser_steal_service_context_table (BusConfigParser *parser);

BusConfigParser* bus_config_load (const DBusString      *file,
                                  dbus_bool_t            is_toplevel,
                                  const BusConfigParser *parent,
                                  DBusError             *error);

#endif /* BUS_CONFIG_PARSER_H */
