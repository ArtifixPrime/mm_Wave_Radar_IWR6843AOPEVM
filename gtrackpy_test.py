from ctypes import byref
import gtrackpy as gt
import ctypes as ct

cfg = gt.GTRACK_moduleConfig()
cfg.deltaT = 1
i = ct.c_int(0)
m = gt.gtrack_create(byref(cfg), byref(i))
gt.gtrack_delete(m)
