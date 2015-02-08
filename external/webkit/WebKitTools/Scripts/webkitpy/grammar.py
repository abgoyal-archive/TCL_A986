#!/usr/bin/env python
import re


def plural(noun):
    # This is a dumb plural() implementation that is just enough for our uses.
    if re.search("h$", noun):
        return noun + "es"
    else:
        return noun + "s"


def pluralize(noun, count):
    if count != 1:
        noun = plural(noun)
    return "%d %s" % (count, noun)


def join_with_separators(list_of_strings, separator=', ', last_separator=', and '):
    if not list_of_strings:
        return ""
    if len(list_of_strings) == 1:
        return list_of_strings[0]
    return "%s%s%s" % (separator.join(list_of_strings[:-1]), last_separator, list_of_strings[-1])
