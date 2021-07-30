r"""Wrapper for gtrack.h

Generated with:
/home/ak/.local/bin/ctypesgen -l gtrack.so gtrack.h

Do not modify this file.
"""

__docformat__ = "restructuredtext"

# Begin preamble for Python v(3, 2)

import ctypes, os, sys
from ctypes import *

_int_types = (c_int16, c_int32)
if hasattr(ctypes, "c_int64"):
    # Some builds of ctypes apparently do not have c_int64
    # defined; it's a pretty good bet that these builds do not
    # have 64-bit pointers.
    _int_types += (c_int64,)
for t in _int_types:
    if sizeof(t) == sizeof(c_size_t):
        c_ptrdiff_t = t
del t
del _int_types


class UserString:
    def __init__(self, seq):
        if isinstance(seq, bytes):
            self.data = seq
        elif isinstance(seq, UserString):
            self.data = seq.data[:]
        else:
            self.data = str(seq).encode()

    def __bytes__(self):
        return self.data

    def __str__(self):
        return self.data.decode()

    def __repr__(self):
        return repr(self.data)

    def __int__(self):
        return int(self.data.decode())

    def __long__(self):
        return int(self.data.decode())

    def __float__(self):
        return float(self.data.decode())

    def __complex__(self):
        return complex(self.data.decode())

    def __hash__(self):
        return hash(self.data)

    def __cmp__(self, string):
        if isinstance(string, UserString):
            return cmp(self.data, string.data)
        else:
            return cmp(self.data, string)

    def __le__(self, string):
        if isinstance(string, UserString):
            return self.data <= string.data
        else:
            return self.data <= string

    def __lt__(self, string):
        if isinstance(string, UserString):
            return self.data < string.data
        else:
            return self.data < string

    def __ge__(self, string):
        if isinstance(string, UserString):
            return self.data >= string.data
        else:
            return self.data >= string

    def __gt__(self, string):
        if isinstance(string, UserString):
            return self.data > string.data
        else:
            return self.data > string

    def __eq__(self, string):
        if isinstance(string, UserString):
            return self.data == string.data
        else:
            return self.data == string

    def __ne__(self, string):
        if isinstance(string, UserString):
            return self.data != string.data
        else:
            return self.data != string

    def __contains__(self, char):
        return char in self.data

    def __len__(self):
        return len(self.data)

    def __getitem__(self, index):
        return self.__class__(self.data[index])

    def __getslice__(self, start, end):
        start = max(start, 0)
        end = max(end, 0)
        return self.__class__(self.data[start:end])

    def __add__(self, other):
        if isinstance(other, UserString):
            return self.__class__(self.data + other.data)
        elif isinstance(other, bytes):
            return self.__class__(self.data + other)
        else:
            return self.__class__(self.data + str(other).encode())

    def __radd__(self, other):
        if isinstance(other, bytes):
            return self.__class__(other + self.data)
        else:
            return self.__class__(str(other).encode() + self.data)

    def __mul__(self, n):
        return self.__class__(self.data * n)

    __rmul__ = __mul__

    def __mod__(self, args):
        return self.__class__(self.data % args)

    # the following methods are defined in alphabetical order:
    def capitalize(self):
        return self.__class__(self.data.capitalize())

    def center(self, width, *args):
        return self.__class__(self.data.center(width, *args))

    def count(self, sub, start=0, end=sys.maxsize):
        return self.data.count(sub, start, end)

    def decode(self, encoding=None, errors=None):  # XXX improve this?
        if encoding:
            if errors:
                return self.__class__(self.data.decode(encoding, errors))
            else:
                return self.__class__(self.data.decode(encoding))
        else:
            return self.__class__(self.data.decode())

    def encode(self, encoding=None, errors=None):  # XXX improve this?
        if encoding:
            if errors:
                return self.__class__(self.data.encode(encoding, errors))
            else:
                return self.__class__(self.data.encode(encoding))
        else:
            return self.__class__(self.data.encode())

    def endswith(self, suffix, start=0, end=sys.maxsize):
        return self.data.endswith(suffix, start, end)

    def expandtabs(self, tabsize=8):
        return self.__class__(self.data.expandtabs(tabsize))

    def find(self, sub, start=0, end=sys.maxsize):
        return self.data.find(sub, start, end)

    def index(self, sub, start=0, end=sys.maxsize):
        return self.data.index(sub, start, end)

    def isalpha(self):
        return self.data.isalpha()

    def isalnum(self):
        return self.data.isalnum()

    def isdecimal(self):
        return self.data.isdecimal()

    def isdigit(self):
        return self.data.isdigit()

    def islower(self):
        return self.data.islower()

    def isnumeric(self):
        return self.data.isnumeric()

    def isspace(self):
        return self.data.isspace()

    def istitle(self):
        return self.data.istitle()

    def isupper(self):
        return self.data.isupper()

    def join(self, seq):
        return self.data.join(seq)

    def ljust(self, width, *args):
        return self.__class__(self.data.ljust(width, *args))

    def lower(self):
        return self.__class__(self.data.lower())

    def lstrip(self, chars=None):
        return self.__class__(self.data.lstrip(chars))

    def partition(self, sep):
        return self.data.partition(sep)

    def replace(self, old, new, maxsplit=-1):
        return self.__class__(self.data.replace(old, new, maxsplit))

    def rfind(self, sub, start=0, end=sys.maxsize):
        return self.data.rfind(sub, start, end)

    def rindex(self, sub, start=0, end=sys.maxsize):
        return self.data.rindex(sub, start, end)

    def rjust(self, width, *args):
        return self.__class__(self.data.rjust(width, *args))

    def rpartition(self, sep):
        return self.data.rpartition(sep)

    def rstrip(self, chars=None):
        return self.__class__(self.data.rstrip(chars))

    def split(self, sep=None, maxsplit=-1):
        return self.data.split(sep, maxsplit)

    def rsplit(self, sep=None, maxsplit=-1):
        return self.data.rsplit(sep, maxsplit)

    def splitlines(self, keepends=0):
        return self.data.splitlines(keepends)

    def startswith(self, prefix, start=0, end=sys.maxsize):
        return self.data.startswith(prefix, start, end)

    def strip(self, chars=None):
        return self.__class__(self.data.strip(chars))

    def swapcase(self):
        return self.__class__(self.data.swapcase())

    def title(self):
        return self.__class__(self.data.title())

    def translate(self, *args):
        return self.__class__(self.data.translate(*args))

    def upper(self):
        return self.__class__(self.data.upper())

    def zfill(self, width):
        return self.__class__(self.data.zfill(width))


class MutableString(UserString):
    """mutable string objects

    Python strings are immutable objects.  This has the advantage, that
    strings may be used as dictionary keys.  If this property isn't needed
    and you insist on changing string values in place instead, you may cheat
    and use MutableString.

    But the purpose of this class is an educational one: to prevent
    people from inventing their own mutable string class derived
    from UserString and than forget thereby to remove (override) the
    __hash__ method inherited from UserString.  This would lead to
    errors that would be very hard to track down.

    A faster and better solution is to rewrite your program using lists."""

    def __init__(self, string=""):
        self.data = string

    def __hash__(self):
        raise TypeError("unhashable type (it is mutable)")

    def __setitem__(self, index, sub):
        if index < 0:
            index += len(self.data)
        if index < 0 or index >= len(self.data):
            raise IndexError
        self.data = self.data[:index] + sub + self.data[index + 1 :]

    def __delitem__(self, index):
        if index < 0:
            index += len(self.data)
        if index < 0 or index >= len(self.data):
            raise IndexError
        self.data = self.data[:index] + self.data[index + 1 :]

    def __setslice__(self, start, end, sub):
        start = max(start, 0)
        end = max(end, 0)
        if isinstance(sub, UserString):
            self.data = self.data[:start] + sub.data + self.data[end:]
        elif isinstance(sub, bytes):
            self.data = self.data[:start] + sub + self.data[end:]
        else:
            self.data = self.data[:start] + str(sub).encode() + self.data[end:]

    def __delslice__(self, start, end):
        start = max(start, 0)
        end = max(end, 0)
        self.data = self.data[:start] + self.data[end:]

    def immutable(self):
        return UserString(self.data)

    def __iadd__(self, other):
        if isinstance(other, UserString):
            self.data += other.data
        elif isinstance(other, bytes):
            self.data += other
        else:
            self.data += str(other).encode()
        return self

    def __imul__(self, n):
        self.data *= n
        return self


class String(MutableString, Union):

    _fields_ = [("raw", POINTER(c_char)), ("data", c_char_p)]

    def __init__(self, obj=""):
        if isinstance(obj, (bytes, UserString)):
            self.data = bytes(obj)
        else:
            self.raw = obj

    def __len__(self):
        return self.data and len(self.data) or 0

    def from_param(cls, obj):
        # Convert None or 0
        if obj is None or obj == 0:
            return cls(POINTER(c_char)())

        # Convert from String
        elif isinstance(obj, String):
            return obj

        # Convert from bytes
        elif isinstance(obj, bytes):
            return cls(obj)

        # Convert from str
        elif isinstance(obj, str):
            return cls(obj.encode())

        # Convert from c_char_p
        elif isinstance(obj, c_char_p):
            return obj

        # Convert from POINTER(c_char)
        elif isinstance(obj, POINTER(c_char)):
            return obj

        # Convert from raw pointer
        elif isinstance(obj, int):
            return cls(cast(obj, POINTER(c_char)))

        # Convert from c_char array
        elif isinstance(obj, c_char * len(obj)):
            return obj

        # Convert from object
        else:
            return String.from_param(obj._as_parameter_)

    from_param = classmethod(from_param)


def ReturnString(obj, func=None, arguments=None):
    return String.from_param(obj)


# As of ctypes 1.0, ctypes does not support custom error-checking
# functions on callbacks, nor does it support custom datatypes on
# callbacks, so we must ensure that all callbacks return
# primitive datatypes.
#
# Non-primitive return values wrapped with UNCHECKED won't be
# typechecked, and will be converted to c_void_p.
def UNCHECKED(type):
    if hasattr(type, "_type_") and isinstance(type._type_, str) and type._type_ != "P":
        return type
    else:
        return c_void_p


# ctypes doesn't have direct support for variadic functions, so we have to write
# our own wrapper class
class _variadic_function(object):
    def __init__(self, func, restype, argtypes, errcheck):
        self.func = func
        self.func.restype = restype
        self.argtypes = argtypes
        if errcheck:
            self.func.errcheck = errcheck

    def _as_parameter_(self):
        # So we can pass this variadic function as a function pointer
        return self.func

    def __call__(self, *args):
        fixed_args = []
        i = 0
        for argtype in self.argtypes:
            # Typecheck what we can
            fixed_args.append(argtype.from_param(args[i]))
            i += 1
        return self.func(*fixed_args + list(args[i:]))


def ord_if_char(value):
    """
    Simple helper used for casts to simple builtin types:  if the argument is a
    string type, it will be converted to it's ordinal value.

    This function will raise an exception if the argument is string with more
    than one characters.
    """
    return ord(value) if (isinstance(value, bytes) or isinstance(value, str)) else value

# End preamble

_libs = {}
_libdirs = []

# Begin loader

# ----------------------------------------------------------------------------
# Copyright (c) 2008 David James
# Copyright (c) 2006-2008 Alex Holkner
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of pyglet nor the names of its
#    contributors may be used to endorse or promote products
#    derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

import os.path, re, sys, glob
import platform
import ctypes
import ctypes.util


def _environ_path(name):
    if name in os.environ:
        return os.environ[name].split(":")
    else:
        return []


class LibraryLoader(object):
    # library names formatted specifically for platforms
    name_formats = ["%s"]

    class Lookup(object):
        mode = ctypes.DEFAULT_MODE

        def __init__(self, path):
            super(LibraryLoader.Lookup, self).__init__()
            self.access = dict(cdecl=ctypes.CDLL(path, self.mode))

        def get(self, name, calling_convention="cdecl"):
            if calling_convention not in self.access:
                raise LookupError(
                    "Unknown calling convention '{}' for function '{}'".format(
                        calling_convention, name
                    )
                )
            return getattr(self.access[calling_convention], name)

        def has(self, name, calling_convention="cdecl"):
            if calling_convention not in self.access:
                return False
            return hasattr(self.access[calling_convention], name)

        def __getattr__(self, name):
            return getattr(self.access["cdecl"], name)

    def __init__(self):
        self.other_dirs = []

    def __call__(self, libname):
        """Given the name of a library, load it."""
        paths = self.getpaths(libname)

        for path in paths:
            try:
                return self.Lookup(path)
            except:
                pass

        raise ImportError("Could not load %s." % libname)

    def getpaths(self, libname):
        """Return a list of paths where the library might be found."""
        if os.path.isabs(libname):
            yield libname
        else:
            # search through a prioritized series of locations for the library

            # we first search any specific directories identified by user
            for dir_i in self.other_dirs:
                for fmt in self.name_formats:
                    # dir_i should be absolute already
                    yield os.path.join(dir_i, fmt % libname)

            # then we search the directory where the generated python interface is stored
            for fmt in self.name_formats:
                yield os.path.abspath(os.path.join(os.path.dirname(__file__), fmt % libname))

            # now, use the ctypes tools to try to find the library
            for fmt in self.name_formats:
                path = ctypes.util.find_library(fmt % libname)
                if path:
                    yield path

            # then we search all paths identified as platform-specific lib paths
            for path in self.getplatformpaths(libname):
                yield path

            # Finally, we'll try the users current working directory
            for fmt in self.name_formats:
                yield os.path.abspath(os.path.join(os.path.curdir, fmt % libname))

    def getplatformpaths(self, libname):
        return []


# Darwin (Mac OS X)


class DarwinLibraryLoader(LibraryLoader):
    name_formats = [
        "lib%s.dylib",
        "lib%s.so",
        "lib%s.bundle",
        "%s.dylib",
        "%s.so",
        "%s.bundle",
        "%s",
    ]

    class Lookup(LibraryLoader.Lookup):
        # Darwin requires dlopen to be called with mode RTLD_GLOBAL instead
        # of the default RTLD_LOCAL.  Without this, you end up with
        # libraries not being loadable, resulting in "Symbol not found"
        # errors
        mode = ctypes.RTLD_GLOBAL

    def getplatformpaths(self, libname):
        if os.path.pathsep in libname:
            names = [libname]
        else:
            names = [format % libname for format in self.name_formats]

        for dir in self.getdirs(libname):
            for name in names:
                yield os.path.join(dir, name)

    def getdirs(self, libname):
        """Implements the dylib search as specified in Apple documentation:

        http://developer.apple.com/documentation/DeveloperTools/Conceptual/
            DynamicLibraries/Articles/DynamicLibraryUsageGuidelines.html

        Before commencing the standard search, the method first checks
        the bundle's ``Frameworks`` directory if the application is running
        within a bundle (OS X .app).
        """

        dyld_fallback_library_path = _environ_path("DYLD_FALLBACK_LIBRARY_PATH")
        if not dyld_fallback_library_path:
            dyld_fallback_library_path = [os.path.expanduser("~/lib"), "/usr/local/lib", "/usr/lib"]

        dirs = []

        if "/" in libname:
            dirs.extend(_environ_path("DYLD_LIBRARY_PATH"))
        else:
            dirs.extend(_environ_path("LD_LIBRARY_PATH"))
            dirs.extend(_environ_path("DYLD_LIBRARY_PATH"))

        if hasattr(sys, "frozen") and sys.frozen == "macosx_app":
            dirs.append(os.path.join(os.environ["RESOURCEPATH"], "..", "Frameworks"))

        dirs.extend(dyld_fallback_library_path)

        return dirs


# Posix


class PosixLibraryLoader(LibraryLoader):
    _ld_so_cache = None

    _include = re.compile(r"^\s*include\s+(?P<pattern>.*)")

    class _Directories(dict):
        def __init__(self):
            self.order = 0

        def add(self, directory):
            if len(directory) > 1:
                directory = directory.rstrip(os.path.sep)
            # only adds and updates order if exists and not already in set
            if not os.path.exists(directory):
                return
            o = self.setdefault(directory, self.order)
            if o == self.order:
                self.order += 1

        def extend(self, directories):
            for d in directories:
                self.add(d)

        def ordered(self):
            return (i[0] for i in sorted(self.items(), key=lambda D: D[1]))

    def _get_ld_so_conf_dirs(self, conf, dirs):
        """
        Recursive funtion to help parse all ld.so.conf files, including proper
        handling of the `include` directive.
        """

        try:
            with open(conf) as f:
                for D in f:
                    D = D.strip()
                    if not D:
                        continue

                    m = self._include.match(D)
                    if not m:
                        dirs.add(D)
                    else:
                        for D2 in glob.glob(m.group("pattern")):
                            self._get_ld_so_conf_dirs(D2, dirs)
        except IOError:
            pass

    def _create_ld_so_cache(self):
        # Recreate search path followed by ld.so.  This is going to be
        # slow to build, and incorrect (ld.so uses ld.so.cache, which may
        # not be up-to-date).  Used only as fallback for distros without
        # /sbin/ldconfig.
        #
        # We assume the DT_RPATH and DT_RUNPATH binary sections are omitted.

        directories = self._Directories()
        for name in (
            "LD_LIBRARY_PATH",
            "SHLIB_PATH",  # HPUX
            "LIBPATH",  # OS/2, AIX
            "LIBRARY_PATH",  # BE/OS
        ):
            if name in os.environ:
                directories.extend(os.environ[name].split(os.pathsep))

        self._get_ld_so_conf_dirs("/etc/ld.so.conf", directories)

        bitage = platform.architecture()[0]

        unix_lib_dirs_list = []
        if bitage.startswith("64"):
            # prefer 64 bit if that is our arch
            unix_lib_dirs_list += ["/lib64", "/usr/lib64"]

        # must include standard libs, since those paths are also used by 64 bit
        # installs
        unix_lib_dirs_list += ["/lib", "/usr/lib"]
        if sys.platform.startswith("linux"):
            # Try and support multiarch work in Ubuntu
            # https://wiki.ubuntu.com/MultiarchSpec
            if bitage.startswith("32"):
                # Assume Intel/AMD x86 compat
                unix_lib_dirs_list += ["/lib/i386-linux-gnu", "/usr/lib/i386-linux-gnu"]
            elif bitage.startswith("64"):
                # Assume Intel/AMD x86 compat
                unix_lib_dirs_list += ["/lib/x86_64-linux-gnu", "/usr/lib/x86_64-linux-gnu"]
            else:
                # guess...
                unix_lib_dirs_list += glob.glob("/lib/*linux-gnu")
        directories.extend(unix_lib_dirs_list)

        cache = {}
        lib_re = re.compile(r"lib(.*)\.s[ol]")
        ext_re = re.compile(r"\.s[ol]$")
        for dir in directories.ordered():
            try:
                for path in glob.glob("%s/*.s[ol]*" % dir):
                    file = os.path.basename(path)

                    # Index by filename
                    cache_i = cache.setdefault(file, set())
                    cache_i.add(path)

                    # Index by library name
                    match = lib_re.match(file)
                    if match:
                        library = match.group(1)
                        cache_i = cache.setdefault(library, set())
                        cache_i.add(path)
            except OSError:
                pass

        self._ld_so_cache = cache

    def getplatformpaths(self, libname):
        if self._ld_so_cache is None:
            self._create_ld_so_cache()

        result = self._ld_so_cache.get(libname, set())
        for i in result:
            # we iterate through all found paths for library, since we may have
            # actually found multiple architectures or other library types that
            # may not load
            yield i


# Windows


class WindowsLibraryLoader(LibraryLoader):
    name_formats = ["%s.dll", "lib%s.dll", "%slib.dll", "%s"]

    class Lookup(LibraryLoader.Lookup):
        def __init__(self, path):
            super(WindowsLibraryLoader.Lookup, self).__init__(path)
            self.access["stdcall"] = ctypes.windll.LoadLibrary(path)


# Platform switching

# If your value of sys.platform does not appear in this dict, please contact
# the Ctypesgen maintainers.

loaderclass = {
    "darwin": DarwinLibraryLoader,
    "cygwin": WindowsLibraryLoader,
    "win32": WindowsLibraryLoader,
    "msys": WindowsLibraryLoader,
}

load_library = loaderclass.get(sys.platform, PosixLibraryLoader)()


def add_library_search_dirs(other_dirs):
    """
    Add libraries to search paths.
    If library paths are relative, convert them to absolute with respect to this
    file's directory
    """
    for F in other_dirs:
        if not os.path.isabs(F):
            F = os.path.abspath(F)
        load_library.other_dirs.append(F)


del loaderclass

# End loader

add_library_search_dirs([])

# Begin libraries
_libs["gtrack.so"] = load_library("gtrack.so")

# 1 libraries
# End libraries

# No modules

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/include/gtrack_2d.h: 63
class struct_anon_3(Structure):
    pass

struct_anon_3.__slots__ = [
    'range',
    'angle',
    'doppler',
]
struct_anon_3._fields_ = [
    ('range', c_float),
    ('angle', c_float),
    ('doppler', c_float),
]

GTRACK_measurement_vector = struct_anon_3# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/include/gtrack_2d.h: 63

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/include/gtrack_2d.h: 117
class struct_anon_6(Structure):
    pass

struct_anon_6.__slots__ = [
    'posX',
    'posY',
    'velX',
    'velY',
    'accX',
    'accY',
]
struct_anon_6._fields_ = [
    ('posX', c_float),
    ('posY', c_float),
    ('velX', c_float),
    ('velY', c_float),
    ('accX', c_float),
    ('accY', c_float),
]

GTRACK_state_vector_pos_vel_acc = struct_anon_6# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/include/gtrack_2d.h: 117

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 308
class struct_anon_7(Structure):
    pass

struct_anon_7.__slots__ = [
    'x1',
    'x2',
    'y1',
    'y2',
    'z1',
    'z2',
]
struct_anon_7._fields_ = [
    ('x1', c_float),
    ('x2', c_float),
    ('y1', c_float),
    ('y2', c_float),
    ('z1', c_float),
    ('z2', c_float),
]

GTRACK_boundaryBox = struct_anon_7# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 308

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 325
class struct_anon_8(Structure):
    pass

struct_anon_8.__slots__ = [
    'depth',
    'width',
    'height',
    'vel',
]
struct_anon_8._fields_ = [
    ('depth', c_float),
    ('width', c_float),
    ('height', c_float),
    ('vel', c_float),
]

GTRACK_gateLimits = struct_anon_8# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 325

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 344
class struct_anon_9(Structure):
    pass

struct_anon_9.__slots__ = [
    'widthStd',
    'depthStd',
    'heightStd',
    'dopplerStd',
]
struct_anon_9._fields_ = [
    ('widthStd', c_float),
    ('depthStd', c_float),
    ('heightStd', c_float),
    ('dopplerStd', c_float),
]

GTRACK_varParams = struct_anon_9# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 344

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 365
class struct_anon_10(Structure):
    pass

struct_anon_10.__slots__ = [
    'numBoundaryBoxes',
    'boundaryBox',
    'numStaticBoxes',
    'staticBox',
]
struct_anon_10._fields_ = [
    ('numBoundaryBoxes', c_uint8),
    ('boundaryBox', GTRACK_boundaryBox * int(2)),
    ('numStaticBoxes', c_uint8),
    ('staticBox', GTRACK_boundaryBox * int(2)),
]

GTRACK_sceneryParams = struct_anon_10# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 365

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 379
class union_anon_11(Union):
    pass

union_anon_11.__slots__ = [
    'limits',
    'limitsArray',
]
union_anon_11._fields_ = [
    ('limits', GTRACK_gateLimits),
    ('limitsArray', c_float * int(4)),
]

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 383
class struct_anon_12(Structure):
    pass

struct_anon_12.__slots__ = [
    'gain',
    'unnamed_1',
]
struct_anon_12._anonymous_ = [
    'unnamed_1',
]
struct_anon_12._fields_ = [
    ('gain', c_float),
    ('unnamed_1', union_anon_11),
]

GTRACK_gatingParams = struct_anon_12# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 383

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 404
class struct_anon_13(Structure):
    pass

struct_anon_13.__slots__ = [
    'det2actThre',
    'det2freeThre',
    'active2freeThre',
    'static2freeThre',
    'exit2freeThre',
]
struct_anon_13._fields_ = [
    ('det2actThre', c_uint16),
    ('det2freeThre', c_uint16),
    ('active2freeThre', c_uint16),
    ('static2freeThre', c_uint16),
    ('exit2freeThre', c_uint16),
]

GTRACK_stateParams = struct_anon_13# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 404

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 427
class struct_anon_14(Structure):
    pass

struct_anon_14.__slots__ = [
    'snrThre',
    'snrThreObscured',
    'velocityThre',
    'pointsThre',
    'maxDistanceThre',
    'maxVelThre',
]
struct_anon_14._fields_ = [
    ('snrThre', c_float),
    ('snrThreObscured', c_float),
    ('velocityThre', c_float),
    ('pointsThre', c_uint16),
    ('maxDistanceThre', c_float),
    ('maxVelThre', c_float),
]

GTRACK_allocationParams = struct_anon_14# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 427

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 441
class struct_anon_15(Structure):
    pass

struct_anon_15.__slots__ = [
    'alpha',
    'confidence',
]
struct_anon_15._fields_ = [
    ('alpha', c_float),
    ('confidence', c_float),
]

GTRACK_unrollingParams = struct_anon_15# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 441

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 464
class struct_anon_16(Structure):
    pass

struct_anon_16.__slots__ = [
    'gatingParams',
    'allocationParams',
    'unrollingParams',
    'stateParams',
    'variationParams',
    'sceneryParams',
]
struct_anon_16._fields_ = [
    ('gatingParams', POINTER(GTRACK_gatingParams)),
    ('allocationParams', POINTER(GTRACK_allocationParams)),
    ('unrollingParams', POINTER(GTRACK_unrollingParams)),
    ('stateParams', POINTER(GTRACK_stateParams)),
    ('variationParams', POINTER(GTRACK_varParams)),
    ('sceneryParams', POINTER(GTRACK_sceneryParams)),
]

GTRACK_advancedParameters = struct_anon_16# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 464

enum_anon_17 = c_int# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 488

GTRACK_STATE_VECTORS_2DV = 0# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 488

GTRACK_STATE_VECTORS_2DA = (GTRACK_STATE_VECTORS_2DV + 1)# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 488

GTRACK_STATE_VECTORS_3DV = (GTRACK_STATE_VECTORS_2DA + 1)# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 488

GTRACK_STATE_VECTORS_3DA = (GTRACK_STATE_VECTORS_3DV + 1)# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 488

GTRACK_STATE_VECTOR_TYPE = enum_anon_17# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 488

enum_anon_18 = c_int# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 512

GTRACK_VERBOSE_NONE = 0# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 512

GTRACK_VERBOSE_ERROR = (GTRACK_VERBOSE_NONE + 1)# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 512

GTRACK_VERBOSE_WARNING = (GTRACK_VERBOSE_ERROR + 1)# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 512

GTRACK_VERBOSE_DEBUG = (GTRACK_VERBOSE_WARNING + 1)# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 512

GTRACK_VERBOSE_MATRIX = (GTRACK_VERBOSE_DEBUG + 1)# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 512

GTRACK_VERBOSE_MAXIMUM = (GTRACK_VERBOSE_MATRIX + 1)# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 512

GTRACK_VERBOSE_TYPE = enum_anon_18# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 512

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 547
class struct_anon_19(Structure):
    pass

struct_anon_19.__slots__ = [
    'stateVectorType',
    'verbose',
    'maxNumPoints',
    'maxNumTracks',
    'initialRadialVelocity',
    'maxRadialVelocity',
    'radialVelocityResolution',
    'maxAcceleration',
    'deltaT',
    'advParams',
]
struct_anon_19._fields_ = [
    ('stateVectorType', GTRACK_STATE_VECTOR_TYPE),
    ('verbose', GTRACK_VERBOSE_TYPE),
    ('maxNumPoints', c_uint16),
    ('maxNumTracks', c_uint16),
    ('initialRadialVelocity', c_float),
    ('maxRadialVelocity', c_float),
    ('radialVelocityResolution', c_float),
    ('maxAcceleration', c_float * int(3)),
    ('deltaT', c_float),
    ('advParams', POINTER(GTRACK_advancedParameters)),
]

GTRACK_moduleConfig = struct_anon_19# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 547

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 561
class union_anon_20(Union):
    pass

union_anon_20.__slots__ = [
    'vector',
    'array',
]
union_anon_20._fields_ = [
    ('vector', GTRACK_measurement_vector),
    ('array', c_float * int((sizeof(GTRACK_measurement_vector) / sizeof(c_float)))),
]

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 568
class struct_anon_21(Structure):
    pass

struct_anon_21.__slots__ = [
    'unnamed_1',
    'snr',
]
struct_anon_21._anonymous_ = [
    'unnamed_1',
]
struct_anon_21._fields_ = [
    ('unnamed_1', union_anon_20),
    ('snr', c_float),
]

GTRACK_measurementPoint = struct_anon_21# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 568

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 574
class union_anon_22(Union):
    pass

union_anon_22.__slots__ = [
    'vector',
    'array',
]
union_anon_22._fields_ = [
    ('vector', GTRACK_measurement_vector),
    ('array', c_float * int((sizeof(GTRACK_measurement_vector) / sizeof(c_float)))),
]

GTRACK_measurementUnion = union_anon_22# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 574

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 599
class struct_anon_23(Structure):
    pass

struct_anon_23.__slots__ = [
    'uid',
    'tid',
    'S',
    'EC',
    'G',
    'dim',
]
struct_anon_23._fields_ = [
    ('uid', c_uint8),
    ('tid', c_uint32),
    ('S', c_float * int((sizeof(GTRACK_state_vector_pos_vel_acc) / sizeof(c_float)))),
    ('EC', c_float * int((((sizeof(GTRACK_measurement_vector) / sizeof(c_float)) * sizeof(GTRACK_measurement_vector)) / sizeof(c_float)))),
    ('G', c_float),
    ('dim', c_float * int((sizeof(GTRACK_measurement_vector) / sizeof(c_float)))),
]

GTRACK_targetDesc = struct_anon_23# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 599

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 601
if _libs["gtrack.so"].has("gtrack_create", "cdecl"):
    gtrack_create = _libs["gtrack.so"].get("gtrack_create", "cdecl")
    gtrack_create.argtypes = [POINTER(GTRACK_moduleConfig), POINTER(c_int32)]
    gtrack_create.restype = POINTER(c_ubyte)
    gtrack_create.errcheck = lambda v,*a : cast(v, c_void_p)

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 602
for _lib in _libs.values():
    if not _lib.has("gtrack_step", "cdecl"):
        continue
    gtrack_step = _lib.get("gtrack_step", "cdecl")
    gtrack_step.argtypes = [POINTER(None), POINTER(GTRACK_measurementPoint), POINTER(GTRACK_measurement_vector), c_uint16, POINTER(GTRACK_targetDesc), POINTER(c_uint16), POINTER(c_uint8), POINTER(c_uint32)]
    gtrack_step.restype = None
    break

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 603
if _libs["gtrack.so"].has("gtrack_delete", "cdecl"):
    gtrack_delete = _libs["gtrack.so"].get("gtrack_delete", "cdecl")
    gtrack_delete.argtypes = [POINTER(None)]
    gtrack_delete.restype = None

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 621
if _libs["gtrack.so"].has("gtrack_alloc", "cdecl"):
    gtrack_alloc = _libs["gtrack.so"].get("gtrack_alloc", "cdecl")
    gtrack_alloc.argtypes = [c_uint32, c_uint32]
    gtrack_alloc.restype = POINTER(c_ubyte)
    gtrack_alloc.errcheck = lambda v,*a : cast(v, c_void_p)

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 632
if _libs["gtrack.so"].has("gtrack_free", "cdecl"):
    gtrack_free = _libs["gtrack.so"].get("gtrack_free", "cdecl")
    gtrack_free.argtypes = [POINTER(None), c_uint32]
    gtrack_free.restype = None

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 648
for _lib in _libs.values():
    if _lib.has("gtrack_log", "cdecl"):
        _func = _lib.get("gtrack_log", "cdecl")
        _restype = None
        _errcheck = None
        _argtypes = [GTRACK_VERBOSE_TYPE, String]
        gtrack_log = _variadic_function(_func,_restype,_argtypes,_errcheck)

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 203
try:
    GTRACK_ERRNO_BASE = (-8000)
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 208
try:
    GTRACK_EOK = 0
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 213
try:
    GTRACK_EINVAL = (GTRACK_ERRNO_BASE - 1)
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 219
try:
    GTRACK_EINUSE = (GTRACK_ERRNO_BASE - 2)
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 224
try:
    GTRACK_ENOTIMPL = (GTRACK_ERRNO_BASE - 3)
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 229
try:
    GTRACK_ENOMEM = (GTRACK_ERRNO_BASE - 4)
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 240
try:
    GTRACK_NUM_POINTS_MAX = 1000
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 241
try:
    GTRACK_NUM_TRACKS_MAX = 250
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 250
try:
    GTRACK_ID_POINT_TOO_WEAK = 253
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 251
try:
    GTRACK_ID_POINT_BEHIND_THE_WALL = 254
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 252
try:
    GTRACK_ID_POINT_NOT_ASSOCIATED = 255
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 262
try:
    GTRACK_BENCHMARK_SETUP = 0
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 263
try:
    GTRACK_BENCHMARK_PREDICT = 1
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 264
try:
    GTRACK_BENCHMARK_ASSOCIATE = 2
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 265
try:
    GTRACK_BENCHMARK_ALLOCATE = 3
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 266
try:
    GTRACK_BENCHMARK_UPDATE = 4
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 267
try:
    GTRACK_BENCHMARK_REPORT = 5
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 269
try:
    GTRACK_BENCHMARK_SIZE = GTRACK_BENCHMARK_REPORT
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 284
try:
    GTRACK_MAX_BOUNDARY_BOXES = 2
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 285
try:
    GTRACK_MAX_STATIC_BOXES = 2
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 549
try:
    GTRACK_STATE_VECTOR_SIZE = (sizeof(GTRACK_state_vector_pos_vel_acc) / sizeof(c_float))
except:
    pass

# /home/ak/ext/pnimac-playground/gtrack_algorithm/gtrack_toolbox/gtrack.h: 550
try:
    GTRACK_MEASUREMENT_VECTOR_SIZE = (sizeof(GTRACK_measurement_vector) / sizeof(c_float))
except:
    pass

# No inserted files

# No prefix-stripping

