from enum import IntEnum

class States(IntEnum):
    UNINITIALIZED = 1
    UNDOCKING = 2
    UNDOCKED = 3
    NAVIGATE = 4
    STOP = 5
    STOPPED = 6
    GOHOME = 7
    DOCKING = 8
    DOCKED = 9

class Flags(IntEnum):
    OK = 1
    ERROR = 2