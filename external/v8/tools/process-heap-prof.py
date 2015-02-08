#!/usr/bin/env python
import csv, sys, time, optparse

def ProcessLogFile(filename, options):
  if options.js_cons_profile:
    itemname = 'heap-js-cons-item'
  else:
    itemname = 'heap-sample-item'
  
  first_call_time = None
  sample_time = 0.0
  sampling = False
  try:
    logfile = open(filename, 'rb')
    try:
      logreader = csv.reader(logfile)

      print('JOB "v8"')
      print('DATE "%s"' % time.asctime(time.localtime()))
      print('SAMPLE_UNIT "seconds"')
      print('VALUE_UNIT "bytes"')

      for row in logreader:
        if row[0] == 'heap-sample-begin' and row[1] == 'Heap':
          sample_time = float(row[3])/1000.0
          if first_call_time == None:
            first_call_time = sample_time
          sample_time -= first_call_time
          print('BEGIN_SAMPLE %.2f' % sample_time)
          sampling = True
        elif row[0] == 'heap-sample-end' and row[1] == 'Heap':
          print('END_SAMPLE %.2f' % sample_time)
          sampling = False
        elif row[0] == itemname and sampling:
          print(row[1]),
          if options.count:
            print('%d' % (int(row[2]))),
          if options.size:
            print('%d' % (int(row[3]))),
          print
    finally:
      logfile.close()
  except:
    sys.exit('can\'t open %s' % filename)


def BuildOptions():
  result = optparse.OptionParser()
  result.add_option("--js_cons_profile", help="Constructor profile",
      default=False, action="store_true")
  result.add_option("--size", help="Report object size",
      default=False, action="store_true")
  result.add_option("--count", help="Report object count",
      default=False, action="store_true")
  return result


def ProcessOptions(options):
  if not options.size and not options.count:
    options.size = True
  return True


def Main():
  parser = BuildOptions()
  (options, args) = parser.parse_args()
  if not ProcessOptions(options):
    parser.print_help()
    sys.exit();
  
  if not args:
    print "Missing logfile"
    sys.exit();
    
  ProcessLogFile(args[0], options)


if __name__ == '__main__':
  sys.exit(Main())
