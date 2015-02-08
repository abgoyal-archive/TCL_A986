import re
import urllib2

# Import WebKit-specific modules.
from webkitpy.webkit_logging import log

# WebKit includes a built copy of BeautifulSoup in Scripts/webkitpy
# so this import should always succeed.
from .BeautifulSoup import BeautifulSoup


class BuildBot:

    default_host = "build.webkit.org"

    def __init__(self, host=default_host):
        self.buildbot_host = host
        self.buildbot_server_url = "http://%s/" % self.buildbot_host

        # If any Leopard builder/tester, Windows builder or Chromium builder is
        # red we should not be landing patches.  Other builders should be added
        # to this list once they are known to be reliable.
        # See https://bugs.webkit.org/show_bug.cgi?id=33296 and related bugs.
        self.core_builder_names_regexps = [
            "Leopard",
            "Windows.*Build",
            "Chromium",
        ]

    def _parse_builder_status_from_row(self, status_row):
        # If WebKit's buildbot has an XMLRPC interface we could use, we could
        # do something more sophisticated here.  For now we just parse out the
        # basics, enough to support basic questions like "is the tree green?"
        status_cells = status_row.findAll('td')
        builder = {}

        name_link = status_cells[0].find('a')
        builder['name'] = name_link.string
        # We could generate the builder_url from the name in a future version
        # of this code.
        builder['builder_url'] = self.buildbot_server_url + name_link['href']

        status_link = status_cells[1].find('a')
        if not status_link:
            # We failed to find a link in the first cell, just give up.  This
            # can happen if a builder is just-added, the first cell will just
            # be "no build"
            # Other parts of the code depend on is_green being present.
            builder['is_green'] = False
            return builder
        # Will be either a revision number or a build number
        revision_string = status_link.string
        # If revision_string has non-digits assume it's not a revision number.
        builder['built_revision'] = int(revision_string) \
                                    if not re.match('\D', revision_string) \
                                    else None
        builder['is_green'] = not re.search('fail',
                                            status_cells[1].renderContents())
        # We could parse out the build number instead, but for now just store
        # the URL.
        builder['build_url'] = self.buildbot_server_url + status_link['href']

        # We could parse out the current activity too.

        return builder

    def _builder_statuses_with_names_matching_regexps(self,
                                                      builder_statuses,
                                                      name_regexps):
        builders = []
        for builder in builder_statuses:
            for name_regexp in name_regexps:
                if re.match(name_regexp, builder['name']):
                    builders.append(builder)
        return builders

    def red_core_builders(self):
        red_builders = []
        for builder in self._builder_statuses_with_names_matching_regexps(
                               self.builder_statuses(),
                               self.core_builder_names_regexps):
            if not builder['is_green']:
                red_builders.append(builder)
        return red_builders

    def red_core_builders_names(self):
        red_builders = self.red_core_builders()
        return map(lambda builder: builder['name'], red_builders)

    def core_builders_are_green(self):
        return not self.red_core_builders()

    def builder_statuses(self):
        build_status_url = self.buildbot_server_url + 'one_box_per_builder'
        page = urllib2.urlopen(build_status_url)
        soup = BeautifulSoup(page)

        builders = []
        status_table = soup.find('table')
        for status_row in status_table.findAll('tr'):
            builder = self._parse_builder_status_from_row(status_row)
            builders.append(builder)
        return builders
