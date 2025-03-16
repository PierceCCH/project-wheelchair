import rospy
import threading
import sys
import signal

from sensor_msgs.msg import Joy
from time import sleep
from std_msgs.msg import Int32

from button_callbacks import *
from can2RNET import *

rnet_threads_running = True

### Global variables
joystick_X = 0
joystick_Y = 0
speed_level = 2
battery_level = 0

toggle_filter = False       # Flag to enable/disable CAN filtering. Always initialize to False
filter_can_id = "02000100"  # CAN IDs to be filtered when toggle_filter is ON

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
    Callback for the /joy_input topic to update joystick_X and joystick_Y, as well as handle button presses.

    :param msg: Joy message containing joystick axes
    """
    global joystick_X, joystick_Y, can_mcu
    try:
        print("Received: ", joystick_X, joystick_Y)

        joystick_X = msg.axes[0]
        joystick_Y = msg.axes[1]

    except IndexError:
        rospy.logerr("Joystick axes index out of range")
    
    buttons_state = msg.buttons
    for button, function in button_functions.items():
        if buttons_state[button]:
            rospy.loginfo(f"Button {button} pressed: {function}")
            if function == 'increase_speed':
                set_speed_level(can_mcu, speed_level + 1)
            elif function == 'decrease_speed':
                set_speed_level(can_mcu, speed_level - 1)
            elif function == 'horn':
                play_beep(can_mcu)
            elif function == 'toggle_model':
                global toggle_filter
                toggle_filter = not toggle_filter
                if toggle_filter:
                    rospy.loginfo("CAN filtering enabled.")
                else:
                    rospy.loginfo("CAN filtering disabled.")
            else:
                rospy.logerr(f"Button {button} function not implemented")


def forward_can(can_in, can_out, filter_can):
    """
    Parses frames from can_in and forwards them to can_out.

    :param can_in: Socket for receiving frames for forwarding
    :param can_out: Socket for sending frames to module
    """
    global rnet_threads_running, toggle_filter

    while rnet_threads_running and not rospy.is_shutdown():
        try:
            frame, _ = can_in.recvfrom(16)
            parsed_frame = dissect_frame(frame)
            frame_id = parsed_frame.split('#')[0]  # Extract CAN ID
            
            if filter_can:
                print("Filtering can")
                print("Toggle filter: ", toggle_filter)
                if toggle_filter and (frame_id == filter_can_id):
                    rospy.loginfo(f"Filtered frame with ID {frame_id}: {parsed_frame}")
                    continue

            can_out.send(frame)
            rospy.loginfo(f"Forwarding | Frame with ID {frame_id}: {parsed_frame}")
        except Exception as e:
            rospy.logerr(f"Error in forwarding CAN messages: {e}")


def send_spoofed_frame(can_socket, joy_id):
    """
    Publishes joystick CAN frame every 10ms.

    :param can_socket: socket for sending CAN messages
    :param joy_id: Joystick ID, used as a reference for sending joystick CAN messages
    """

    global rnet_threads_running, joystick_X, joystick_Y, toggle_filter

    while rnet_threads_running and not rospy.is_shutdown():
        if toggle_filter:
            joyframe = joy_id + '#' + dec_to_hex(joystick_X, 2) + dec_to_hex(joystick_Y, 2)
            print("Sending joystick_X: ", joystick_X, " joystick_Y: ", joystick_Y)
            cansend(can_socket, joyframe)


def wait_rnet_joystick_frame(can_socket):
    """
    Waits for a frame send by joystick. Extracts the frame ID and returns it.
    If no frame is found within the timeout, 'Err!' is returned.

    :param can_socket: socket for sending and receiving CAN messages

    :return: Joystick frame extendedID
    :throws: TimeoutError
    """
    frameid = ''
    # timeout = time() + 2 # wait 200ms until timeout

    # while frameid[0:3] != '020':  # joystick frame ID (no extended frame)
    #     cf, _ = can_socket.recvfrom(16) # Blocking if no CANBUS traffic
    #     candump_frame = dissect_frame(cf)
    #     frameid = candump_frame.split('#')[0]
        
    #     if time() > timeout:
    #         print("Joystick frame wait timed out...")
    #         return TimeoutError
        
    return "02000100"


def joystick_spoofing(can_jsm, can_mcu):
    """
    Handles R-Net joystick spoofing.

    :param can_jsm: socket for receiving CAN messages from on board joystick
    :param can_mcu: socket for sending CAN messages to wheelchair control module
    """
    global rnet_threads_running

    rospy.loginfo("Waiting for R-net joystick frame...")

    try:
        joystick_ID = wait_rnet_joystick_frame(can_jsm)
        rospy.loginfo(f"Found R-net joystick frame: {joystick_ID}")
        sleep(10) # To make sure JSM fully boots up

        # Start thread to send joystick CAN frames
        spoof_thread = threading.Thread(
            target=send_spoofed_frame, 
            args=(can_mcu, joystick_ID), 
            daemon=True
        )
        # spoof_thread.start()

        rospy.loginfo("Joystick spoofing started.")

    except TimeoutError:
        rospy.logerr("No R-net joystick frame seen within timeout. Aborting...")


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
        joystick_spoofing(
            can_jsm=can_jsm, 
            can_mcu=can_mcu
        )

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
