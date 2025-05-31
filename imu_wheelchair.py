import rospy
import signal, sys, time
import numpy as np
import tf2_ros
import geometry_msgs.msg

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance, Point, Quaternion, Vector3
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from smbus2 import SMBus
from ahrs.filters import Madgwick


# --- Signal handling ---
def signal_handler(sig, frame):
    rospy.loginfo("Shutting down MPU9255 node...")
    try:
        bus.close()
    except:
        pass
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


# Quarternion rotation
def q_rotate(q, v):
    """
    Rotate vector `v` by quaternion `q`.
    q: [w, x, y, z]
    v: [x, y, z]
    """
    q = np.array(q, dtype=np.float64)
    v = np.array(v, dtype=np.float64)

    q_vec = q[1:]
    uv = np.cross(q_vec, v)
    uuv = np.cross(q_vec, uv)
    return v + 2 * (q[0] * uv + uuv)

def q_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


# --- Wake up MPU9255 ---
bus = SMBus(1)
bus.write_byte_data(0x68, 0x6B, 0x00)
time.sleep(0.1)

# --- Initialize IMU ---
mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68,
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_250,
    afs=AFS_2G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ
)
mpu.calibrate() # Calibrate at every startup
mpu.configure()


madgwick = Madgwick()
q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion

# --- Initialize ROS ---
rospy.init_node('mpu9255_node')
imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
rate = rospy.Rate(100)

# --- State variables for integration ---
velocity = np.zeros(3)
position = np.zeros(3)
last_time = rospy.Time.now()

tf_broadcaster = tf2_ros.TransformBroadcaster()

while not rospy.is_shutdown():
    try:
        acc = np.array(mpu.readAccelerometerMaster())
        gyro = np.array(mpu.readGyroscopeMaster())
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
        last_time = now

        # Update orientation
        q = madgwick.updateIMU(q=q, gyr=np.radians(gyro), acc=acc)

        gravity = np.array([0.0, 0.0, 9.81])
        gravity_body = q_rotate(q_conjugate(q), gravity)
        acc_lin_body = acc - gravity_body
        acc_world = q_rotate(q, acc_lin_body)


        # Integrate
        velocity += acc_world * dt
        position += velocity * dt

        # --- Publish IMU ---
        imu_msg = Imu()
        imu_msg.header = Header(stamp=now, frame_id="imu_link")
        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]
        imu_msg.angular_velocity.x = np.radians(gyro[0])
        imu_msg.angular_velocity.y = np.radians(gyro[1])
        imu_msg.angular_velocity.z = np.radians(gyro[2])
        imu_msg.orientation.x = q[1]
        imu_msg.orientation.y = q[2]
        imu_msg.orientation.z = q[3]
        imu_msg.orientation.w = q[0]
        imu_msg.orientation_covariance[0] = 0.01

        imu_pub.publish(imu_msg)

        # --- Publish Odometry ---
        odom_msg = Odometry()
        odom_msg.header = Header(stamp=now, frame_id="odom")
        odom_msg.child_frame_id = "imu_link"

        odom_msg.pose.pose.position = Point(*position)
        odom_msg.pose.pose.orientation = Quaternion(q[1], q[2], q[3], q[0])
        odom_msg.twist.twist.linear = Vector3(*velocity)
        odom_msg.twist.twist.angular = Vector3(*np.radians(gyro))

        # --- TF ---
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "imu_link"

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        tf_broadcaster.sendTransform(t)

        odom_pub.publish(odom_msg)
        rospy.loginfo("Publishing Odometry and Orientation")

    except Exception as e:
        rospy.logwarn(f"IMU error: {e}")
    rate.sleep()
