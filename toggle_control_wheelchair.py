import rospy
import threading
import sys
import signal

from sensor_msgs.msg import Joy
from time import sleep
from std_msgs.msg import Int32

from can2RNET import *

rnet_threads_running = True

### Global variables
joystick_X = 0
joystick_Y = 0
button_states = []
speed_level = 2
battery_level = 0

toggle_filter = False       # Flag to enable/disable CAN filtering. Always initialize to False
joystick_ID = "02000100"  # CAN IDs to be filtered when toggle_filter is ON

### CAN sockets for communication
can_jsm = None      # CAN0: Receives signals from onboard joystick (JSM)
can_mcu = None      # CAN1: Sends signals to wheelchair control module

### Map button index to function name
button_functions = {
    0: 'horn',
    1: 'toggle_model',
    2: 'decrease_speed',
    3: 'increase_speed',
}


def signal_handler(sig, frame):
    """Handles Ctrl+C for a clean shutdown."""
    global rnet_threads_running, can_mcu, can_jsm
    rospy.loginfo("Interrupt signal received. Stopping wheelchair controller...")

    rnet_threads_running = False
    rospy.signal_shutdown("Node closed...")

    if can_mcu:
        can_mcu.close()
        rospy.loginfo("CAN0 socket closed.")
    if can_jsm:
        can_jsm.close()
        rospy.loginfo("CAN1 socket closed.")

    sys.exit(0)


def joy_callback(msg):
    """
    Callback for the /joy_input topic to update joystick_X, joystick_Y and button_states dictionary.

    :param msg: Joy message containing joystick axes
    """
    global joystick_X, joystick_Y, button_states
    try:
        print("Received: ", joystick_X, joystick_Y)

        with lock:
            joystick_X = msg.axes[0]
            joystick_Y = msg.axes[1]
            button_states = msg.buttons

    except IndexError:
        rospy.logerr("Joystick axes index out of range")


def forward_can(can_in, can_out, filter_can):
    """
    Parses frames from can_in and forwards them to can_out.

    :param can_in: Socket for receiving frames for forwarding
    :param can_out: Socket for sending frames to module
    """
    global rnet_threads_running, toggle_filter, joystick_ID

    while rnet_threads_running and not rospy.is_shutdown():
        try:
            frame, _ = can_in.recvfrom(16)
            parsed_frame = dissect_frame(frame)
            frame_id = parsed_frame.split('#')[0]  # Extract CAN ID
            
            if filter_can:
                print("Filtering can")
                print("Toggle filter: ", toggle_filter)

                if toggle_filter and (frame_id == joystick_ID):
                    rospy.loginfo(f"Filtered frame with ID {frame_id}: {parsed_frame}")
                    joyframe = joystick_ID + '#' + dec_to_hex(joystick_X, 2) + dec_to_hex(joystick_Y, 2)
                    print("Sending joystick_X: ", joystick_X, " joystick_Y: ", joystick_Y)
                    frame=build_frame(joyframe)

                    for button, function in button_functions.items():
                        if button_states[button]:
                            rospy.loginfo(f"Button {button} pressed: {function}")
                            try:
                                if function == 'increase_speed':
                                    new_speed_level = speed_level + 1 # TODO: Fix race condition
                                    set_speed_level(can_out, new_speed_level)
                                    # set_speed_level(can_in, new_speed_level)
                                elif function == 'decrease_speed':
                                    new_speed_level = speed_level - 1
                                    set_speed_level(can_out, new_speed_level)
                                    # set_speed_level(can_in, new_speed_level)
                                elif function == 'horn':
                                    play_beep(can_out)
                                else:
                                    rospy.logerr(f"Button {button} function not implemented")
                            except Exception as e:
                                rospy.logerr(f"Error with button function: {e}")

            can_out.send(frame)
            rospy.loginfo(f"Forwarding | Frame with ID {frame_id}: {parsed_frame}")

        except Exception as e:
            rospy.logerr(f"Error in forwarding CAN messages: {e}")


def read_battery_level(can_socket):
    """
    Waits for a frame and reads the battery level of the wheelchair.

    :param can_socket: socket for sending CAN messages

    :returns: battery level in percentage
    """
    frameid = ''

    battery_level = 0

    while frameid[0:5] != '1c0c0':      # Battery level frame ID (no extended frame)
        cf, _ = can_socket.recvfrom(16) # Blocking if no CANBUS traffic
        candump_frame = dissect_frame(cf)
        frameid = candump_frame.split('#')[0]

        if frameid[0:5] != '1c0c0':
            continue
        battery_level = int(candump_frame.split('#')[1], 16)
        
    return battery_level


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


def main():
    global rnet_threads_running, can_mcu, can_jsm

    signal.signal(signal.SIGINT, signal_handler)

    can_jsm = opencansocket(0)   # CAN0 for reading the onboard joystick
    can_mcu = opencansocket(1)   # CAN1 for controlling the wheelchair

    if not can_mcu:
        rospy.logerr("Failed to open CAN out.")
        sys.exit(1)
    if not can_jsm:
        can_mcu.close()
        rospy.logerr("Failed to open CAN in.")
        sys.exit(1)

    rospy.init_node("wheelchair_controller")
    rospy.Subscriber("/joy_input", Joy, joy_callback)
    battery_level_publisher = rospy.Publisher("/battery_level", Int32, queue_size=1)

    ## Start threads for forwarding CAN messages
    jsm_to_mcu = threading.Thread(
        target=forward_can,
        args=(can_jsm, can_mcu, True),
        daemon=True
    )
    mcu_to_jsm = threading.Thread(
        target=forward_can,
        args=(can_mcu, can_jsm, False),
        daemon=True
    )
    jsm_to_mcu.start()
    mcu_to_jsm.start()

    try:
       # while rnet_threads_running and not rospy.is_shutdown():
       #     battery_level = read_battery_level(can_mcu)
       #     battery_level_publisher.publish(battery_level)

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node...")
    finally:
        rnet_threads_running = False
        if can_mcu:
            can_mcu.close()
        if can_jsm:
            can_jsm.close()
        rospy.loginfo("CAN sockets closed.")

if __name__ == "__main__":
    main()
