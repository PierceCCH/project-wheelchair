import rospy
import threading
import sys
import signal

from sensor_msgs.msg import Joy
from can2RNET import *
from time import sleep, time
from std_msgs.msg import Int32

# Global variables for joystick state
joystick_X = 0
joystick_Y = 0
speed_level = 2
battery_level = 0

rnet_threads_running = True
can_socket = None

# Maps button index to function name
button_functions = {
    0: 'horn',
    1: 'toggle_model',
    4: 'decrease_speed',
    5: 'increase_speed',
}


def signal_handler(sig, frame):
    """
    Handles SIGINT (Ctrl+C) to cleanly exit the program.

    :param sig: Signal number.
    :param frame: Current stack frame.
    """
    global rnet_threads_running, can_socket
    rospy.loginfo("Interrupt signal received. Stopping wheelchair controller...")

    # Stop threads & processes
    rnet_threads_running = False
    rospy.signal_shutdown("Node closed...")

    # Ensure CAN socket is closed properly
    if can_socket:
        can_socket.close()
        rospy.loginfo("CAN socket closed.")

    sys.exit(0)


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


def joy_callback(msg):
    """
    Callback for the /joy_input topic to update joystick_X and joystick_Y, as well as handle button presses.

    :param msg: Joy message containing joystick axes
    """
    global joystick_X, joystick_Y
    try:
        joystick_X = msg.axes[0]
        joystick_Y = msg.axes[1]

    except IndexError:
        rospy.logerr("Joystick axes index out of range")
    
    buttons_state = msg.buttons
    for button, function in button_functions.items():
        if buttons_state[button]:
            print(f"Button {button} pressed: {function}")
            if function == 'increase_speed':
                set_speed_level(can_socket, speed_level + 1)
            elif function == 'decrease_speed':
                set_speed_level(can_socket, speed_level - 1)
            elif function == 'horn':
                play_beep(can_socket)
            else:
                print(f"Button {button} function not implemented")


def send_joystick_canframe(can_socket, joy_id):
    """
    Publishes joystick CAN frame every 10ms.

    :param can_socket: socket for sending CAN messages
    :param joy_id: Joystick ID, used as a reference for sending joystick CAN messages
    """
    mintime = 0.01
    nexttime = time() + mintime

    while rnet_threads_running and not rospy.is_shutdown():
        joyframe = joy_id + '#' + dec_to_hex(joystick_X, 2) + dec_to_hex(joystick_Y, 2)
        print("Sending joystick_X: ", joystick_X, " joystick_Y: ", joystick_Y)
        cansend(can_socket, joyframe)

        nexttime += mintime
        t = time()
        if t < nexttime:
            sleep(nexttime - t)
        else:
            nexttime += mintime


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

        
def disable_rnet_joystick(can_socket):
    """
    Send JSM error exploit CAN message to disable on-board joystick.

    :param can_socket: socket for sending CAN messages

    :returns: True if successful, False otherwise
    """
    print("Waiting for JSM heartbeat...")
    canwait(can_socket,"03C30F0F:1FFFFFFF")

    try:
        for _ in range(0,3):
            cansend(can_socket,'0c000000#')
        return True
    except:
        print("Error sending JSM exploit message.")
        return False


def wait_rnet_joystick_frame(can_socket):
    """
    Waits for a frame send by joystick. Extracts the frame ID and returns it.
    If no frame is found within the timeout, 'Err!' is returned.

    :param can_socket: socket for sending and receiving CAN messages

    :return: Joystick frame extendedID
    :throws: TimeoutError
    """
    frameid = ''
    timeout = time() + 0.2 # wait 200ms until timeout

    while frameid[0:3] != '020':  # joystick frame ID (no extended frame)
        cf, _ = can_socket.recvfrom(16) # Blocking if no CANBUS traffic
        candump_frame = dissect_frame(cf)
        frameid = candump_frame.split('#')[0]
        
        if time() > timeout:
            print("Joystick frame wait timed out...")
            return TimeoutError
        
    return frameid


def read_battery_level(can_socket):
    """
    Waits for a frame and reads the battery level of the wheelchair.

    :param can_socket: socket for sending CAN messages

    :returns: battery level in percentage
    """
    frameid = ''
    timeout = time() + 1.1 # wait 1100ms until timeout

    battery_level = 0

    while frameid[0:5] != '1c0c0':  # Battery level frame ID (no extended frame)
        cf, _ = can_socket.recvfrom(16) # Blocking if no CANBUS traffic
        candump_frame = dissect_frame(cf)
        frameid = candump_frame.split('#')[0]

        if frameid[0:5] != '1c0c0':
            continue
        battery_level = int(candump_frame.split('#')[1], 16)
        
    return battery_level


def joystick_spoofing(can_socket):
    """
    Handles R-Net joystick spoofing.

    :param can_socket: socket for sending CAN messages
    """
    global rnet_threads_running

    rospy.loginfo("Waiting for R-net joystick frame...")

    try:
        joystick_ID = wait_rnet_joystick_frame(can_socket)
        # joystick_ID = joystick_ID[:-1] + '1'
        rospy.loginfo(f"Found R-net joystick frame: {joystick_ID}")
        if not disable_rnet_joystick(can_socket):
            rospy.logerr("Failed to disable R-net joystick. Aborting...")
            return

        # Start thread to send joystick CAN frames
        spoof_thread = threading.Thread(
            target=send_joystick_canframe, 
            args=(can_socket, joystick_ID), 
            daemon=True
        )
        spoof_thread.start()

        rospy.loginfo("Joystick spoofing started.")

    except TimeoutError:
        rospy.logerr("No R-net joystick frame seen within timeout. Aborting...")


def play_beep(cansocket):
    """
    Plays a short beep sound on the wheelchair.

    :param cansocket: socket for sending CAN messages

    :return: None
    """
    cansend(cansocket,"0C040100#") # Tun horn on
    cansend(cansocket,"0C040201#") # Delay
    cansend(cansocket,"0C040201#")
    cansend(cansocket,"0C040101#") # Turn horn off


def main():
    global rnet_threads_running
    global can_socket

    signal.signal(signal.SIGINT, signal_handler)
    
    can_socket = opencansocket(0)

    # Initialize ROS node
    rospy.init_node("wheelchair_controller")
    rospy.Subscriber("/joy_input", Joy, joy_callback)
    battery_level_publisher = rospy.Publisher("/battery_level", Int32, queue_size=1)

    try:
        joystick_spoofing(can_socket)

        while rnet_threads_running and not rospy.is_shutdown():
           battery_level = read_battery_level(can_socket)
           battery_level_publisher.publish(battery_level)
           sleep(5)

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node...")
    finally:
        rnet_threads_running = False
        if can_socket:
            can_socket.close()
        rospy.loginfo("CAN socket closed.")


if __name__ == "__main__":
    main()
