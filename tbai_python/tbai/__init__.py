from tbai._C import *  # noqa: F401, F403
from tbai._C import rotations  # noqa: F401
from tbai import _C  # noqa: F401

# Fix repr so classes show as tbai.X instead of tbai._C.X
for _obj in list(vars().values()):
    if isinstance(_obj, type) and _obj.__module__ == "tbai._C":
        _obj.__module__ = __name__
del _obj
