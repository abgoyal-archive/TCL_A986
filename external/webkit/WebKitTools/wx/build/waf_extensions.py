import os
import subprocess
import sys

import Utils

# version of exec_command that handles Windows command lines longer than 32000 chars
def exec_command(s, **kw):
    filename = ''
    if sys.platform.startswith('win') and len(' '.join(s)) > 32000:
        import tempfile
        (fd, filename) = tempfile.mkstemp()
        os.write(fd, ' '.join(s[1:]))
        os.close(fd)
        
        s = [s[0], '@' + filename]
        
    if 'log' in kw:
        kw['stdout'] = kw['stderr'] = kw['log']
        del(kw['log'])
    kw['shell'] = isinstance(s, str)

    def cleanup():
        try:
            if os.path.exists(filename):
                os.remove(filename)
        except:
            pass

    try:
        proc = subprocess.Popen(s, **kw)
        result = proc.wait()
        cleanup()
        return result
        
    except OSError:
        cleanup()
        raise
        
Utils.exec_command = exec_command

# Better performing h_file to keep hashing from consuming lots of time
import stat
def h_file(filename):
    st = os.stat(filename)
    if stat.S_ISDIR(st[stat.ST_MODE]): raise IOError('not a file')
    m = Utils.md5()
    m.update(str(st.st_mtime))
    m.update(str(st.st_size))
    m.update(filename)
    return m.digest()

Utils.h_file = h_file
