#!/usr/bin/env python
"""Simple test client for the port/Driver interface."""

import os
import optparse
import port


def run_tests(port, options, tests):
    # |image_path| is a path to the image capture from the driver.
    image_path = 'image_result.png'
    driver = port.start_driver(image_path, None)
    for t in tests:
        uri = port.filename_to_uri(os.path.join(port.layout_tests_dir(), t))
        print "uri: " + uri
        crash, timeout, checksum, output, err = \
            driver.run_test(uri, int(options.timeout), None)
        print "crash:         " + str(crash)
        print "timeout:       " + str(timeout)
        print "checksum:      " + str(checksum)
        print 'stdout: """'
        print ''.join(output)
        print '"""'
        print 'stderr: """'
        print ''.join(err)
        print '"""'
        print


if __name__ == '__main__':
    optparser = optparse.OptionParser()
    optparser.add_option('-p', '--platform', action='store', default='mac',
                         help='Platform to test (e.g., "mac", "chromium-mac", etc.')
    optparser.add_option('-t', '--target', action='store', default='Release',
                         help='build type ("Debug" or "Release")')
    optparser.add_option('', '--timeout', action='store', default='2000',
                         help='test timeout in milliseconds (2000 by default)')
    optparser.add_option('', '--wrapper', action='store')
    optparser.add_option('', '--no-pixel-tests', action='store_true',
                         default=False,
                         help='disable pixel-to-pixel PNG comparisons')
    options, args = optparser.parse_args()
    p = port.get(options.platform, options)
    run_tests(p, options, args)
