from enum import Enum

class Intents(Enum):
    STOP = 1
    STOW = 2
    SCAN_ROOM = 3
    REACH_TABLE = 4
    MOVE_TO_TABLE = 5
    GRAB_FROM_GROUND = 6
    TEST_LIFT_SMALL = 7

BASE_LENGTH = .34
BASE_WIDTH = .33


TABLE_HEIGHT = 1.1 # avg table height + length of gripper (also max height by chance)
LIFT_MID_HEIGHT = .55  
GROUND_REACH_HEIGHT = 0.2 


ARM_OUT_TO_TABLE_LEN = .3

