import rospy
import threading
import sys
import signal

from sensor_msgs.msg import Joy
from time import sleep
from std_msgs.msg import Int32

from can2RNET import *

rnet_threads_running = True
lock = threading.Lock()

### Global variables
joystick_X = 0
joystick_Y = 0
button_states = [0 for i in range(10)]
speed_level = 0
battery_level = 0

joystick_ID = "02000100"  # CAN IDs to be filtered when toggle_filter is ON

### CAN sockets for communication
can_jsm = None      # CAN0: Receives signals from onboard joystick (JSM)
can_mcu = None      # CAN1: Sends signals to wheelchair control module

### Publishers
battery_level_publisher = None
control_telem_publisher = None

### Map button index to function name
button_functions = {
    0: 'horn',
    1: 'toggle_model', # Not used in this file
    4: 'decrease_speed',
    5: 'increase_speed',
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
    Callback for the /joy_input topic to update joystick_X, joystick_Y and button_state dictionary.

    :param msg: Joy message containing joystick axes
    """
    global joystick_X, joystick_Y, button_states
    try:

        with lock:
            joystick_X = msg.axes[0]
            joystick_Y = msg.axes[1]

            for i in range(len(button_states)):
                if button_states[i]==0 and msg.buttons[i]==1:
                    button_states[i]=1
                if button_states[i]==2 and msg.buttons[i] ==0:
                    button_states[i]=0

            rospy.loginfo(f"Received gamepad input -- Axes: ({joystick_X}, {joystick_Y}) Buttons: {button_states}")

    except IndexError:
        rospy.logerr("Joystick axes index out of range")
    

def forward_can(can_in, can_out, thread_id):
    """
    Parses frames from can_in and forwards them to can_out.

    :param can_in: Socket for receiving frames for forwarding
    :param can_out: Socket for sending frames to module
    :param thread_id: Identify tagged to socket number of can_in
    """
    global rnet_threads_running, toggle_filter

    while rnet_threads_running and not rospy.is_shutdown():
        try:
            frame, _ = can_in.recvfrom(16)
            parsed_frame = dissect_frame(frame)
            frame_id = parsed_frame.split('#')[0]  # Extract CAN ID

            if frame_id[0:5] == "1c0c0":           # Battery level frame ID
                battery_level = int(parsed_frame.split('#')[1], 16)
                battery_level_publisher.publish(battery_level)

            if frame_id == joystick_ID:
                dec =  parsed_frame.split('#')[1]

                input_x = hex_to_dec(dec[:2])   # JSM x_input
                input_y = hex_to_dec(dec[2:])   # JSM y_input

                # Only use gamepad input if JSM is idle.
                if input_x==0 and input_y==0:
                    joyframe = joystick_ID + '#' + dec_to_hex(joystick_X, 2) + dec_to_hex(joystick_Y, 2)
                    rospy.loginfo(f"Sending joystick_X: {joystick_X} joystick_Y: {joystick_Y}")
                    frame=build_frame(joyframe)

                    # Publish gamepad state
                    control_telem = Joy()
                    control_telem.axes = [joystick_X, joystick_Y]
                    control_telem.buttons = button_states
                    control_telem_publisher.publish(control_telem)

                    # Only 1 thread handles buttons to prevent duplicates
                    if thread_id == 0:
                        for button, function in button_functions.items():
                            if button_states[button] == 1:
                                rospy.loginfo(f"Button {button} pressed: {function}")
                                try:
                                    if function == 'increase_speed':
                                        new_speed_level = speed_level + 1
                                        set_speed_level(can_out, new_speed_level)
                                        set_speed_level(can_in, new_speed_level)
                                    elif function == 'decrease_speed':
                                        new_speed_level = speed_level - 1
                                        set_speed_level(can_out, new_speed_level)
                                        set_speed_level(can_in, new_speed_level)
                                    elif function == 'horn':
                                        play_beep(can_out)
                                        play_beep(can_in)
                                    else:
                                        rospy.logerr(f"Button {button} function not implemented")
                                    button_states[button] = 2
                                except Exception as e:
                                    rospy.logerr(f"Error with button function: {e}")
                else:
                    # Publish state of JSM, ignoring button_states
                    try:
                        control_telem = Joy()
                        control_telem.axes = [input_x, input_y]
                        control_telem.buttons = [0 for i in range(10)]
                        control_telem_publisher.publish(control_telem)
                    except Exception as e:
                        rospy.logerr("Error publishing control state")

            parsed_frame = dissect_frame(frame)
            can_out.send(frame)
        except Exception as e:
            rospy.logerr(f"Error in forwarding CAN messages: {e}")            


def hex_to_dec(hex_str):
    """
    Convert hex to decimal and return it as a string with a specific length.

    :param hex_str: Hexadecimal string to convert
    :param dec_len: Length of the decimal string

    :returns: Decimal string
    """
    # Remove leading zeros and convert hex to decimal
    dec = int(hex_str, 16)

    # Ensure the string is of the desired length by adding leading zeros if needed
    return dec
    

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
        rospy.loginfo(f"Setting speed: {level}")

        with lock:
            speed_level = level
        print("Speed level now: ", speed_level)
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
    global rnet_threads_running, can_mcu, can_jsm, battery_level_publisher, control_telem_publisher

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

    battery_level_publisher = rospy.Publisher("/battery_level", Int32, queue_size=1)    # Shows battery level of wheelchair in percentage
    control_telem_publisher = rospy.Publisher("/control_telem", Joy, queue_size=1)      # Joystick and button states, for logging

    ## Start threads for forwarding CAN messages
    jsm_to_mcu = threading.Thread(
        target=forward_can,
        args=(can_jsm, can_mcu, 0),
        daemon=True
    )
    mcu_to_jsm = threading.Thread(
        target=forward_can,
        args=(can_mcu, can_jsm, 1),
        daemon=True
    )
    jsm_to_mcu.start()
    mcu_to_jsm.start()


    try:
        rospy.spin()    # Keep ros node running
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
