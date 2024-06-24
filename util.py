from enum import Enum

class Intent(Enum):
    STOP = 1
    STOW = 2
    SCAN_ROOM = 3
    REACH_TABLE = 4
    MOVE_TO_TABLE = 5
    GRAB_FROM_GROUND = 6
    
