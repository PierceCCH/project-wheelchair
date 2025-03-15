from can2RNET import *


def dec_to_hex(dec, hexlen):
    """
    Convert dec to hex with leading 0s and no '0x' prefix.

    :param dec: decimal number to convert
    :param hexlen: length of the hex string

    :returns: hex string
    """
    h=hex(int(dec))[2:]
    l=len(h)
    if h[l-1]=="L":
        l-=1  #strip the 'L' that python int sticks on
    if h[l-2]=="x":
        h= '0'+hex(int(dec))[1:]
    return ('0' * hexlen + h)[l:l + hexlen]


def set_speed_level(cansocket, level):
    """
    Set the speed level of the wheelchair.
    Levels are 0-4, with 0 being the slowest and 4 being the fastest.

    :param cansocket: socket for sending CAN messages
    :param level: speed level to set
    """
    global speed_level

    if level >= 0 and level < 5:
        speed_level = level
        cansend(cansocket, '0a040100#' + dec_to_hex(level * 25, 2))
    else:
        print('Invalid RNET speed level: ' + str(level))


def play_beep(cansocket):
    """
    Plays a short beep sound on the wheelchair.

    :param cansocket: socket for sending CAN messages

    :return: None
    """
    cansend(cansocket, "0C040100#")
    cansend(cansocket, "0C040201#")
    cansend(cansocket, "0C040201#")
    cansend(cansocket, "0C040101#")