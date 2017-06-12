"""A module to provide jython compatibility for python 2.3 code.

This module is particularly aimed at the use with POSH and does not guarantee
compatibility for its use with any other program. Some of the current
POSH implementation uses particular python 2.3 features (that are also
supported in python 2.4), but which are not supported in the current jython,
as (at the time of this writing) the latest stable jython implements the
python 2.1 language features.

This module differentates between the different language version and
introduces some symbols that might be lacking do to an older language
specification. For it to be used, it has to be imported using
C{from jython_compat import *} as the first import in any module.

The variables that it sets are:
is_jython is set to 0 if we are using python, and 1 if we are using jython.
True and False is set to 1 and 0 respectively, and the method bool() is
defined if the version is before 2.2.1.
"""

import sys

# determine if running jython or not
if sys.platform[:4] == 'java':
    is_jython = 1
else:
    is_jython = 0

# True, False, bool doesn't exist before 2.2.1
# sys.version_info[0] is major, and sys.version_info[1] is minor version
if sys.version_info[0] < 2 or \
   (sys.version_info[0] == 2 and sys.version_info[1] < 2) or \
   (sys.version_info[0] == 2 and sys.version_info[1] == 2 and \
    sys.version_info[2] < 1):
    True = 1
    False = 0
    def bool(x):
        if x:
            return True
        else:
            return False
