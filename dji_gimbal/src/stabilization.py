#!/usr/bin/env python
import roslib

roslib.load_manifest('dji_gimbal')
import rospy
import tf
#from geometry_msgs.msg import Twist
#from hector_uav_msgs.srv import EnableMotors
#from std_msgs.msg import Header

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

pitch_ang = 0

def rcv(imu_data):
    global pitch_ang
    angle_pitch = Float64()
    angle_roll= Float64()
    angle_yaw= Float64()
    quaternion = (
        imu_data.orientation.x,
        imu_data.orientation.y,
        imu_data.orientation.z,
        imu_data.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    #print(euler)
    angle_roll.data = euler[0]
    roll.publish(angle_roll)
    angle_yaw.data = 0
    yaw.publish(angle_yaw)
    angle_pitch.data = -euler[1]+pitch_ang
    pitch.publish(angle_pitch)


def pitch_rcv(data):
    global pitch_ang
    pitch_ang = data.data

if __name__ == "__main__":
    rospy.init_node('stab')
    yaw = rospy.Publisher('dji_gimbal/yaw_position_controller/command', Float64, queue_size=10)
    pitch = rospy.Publisher('dji_gimbal/pitch_position_controller/command', Float64, queue_size=10)
    roll = rospy.Publisher('dji_gimbal/roll_position_controller/command', Float64, queue_size=10)
    rospy.Subscriber("/mavros/imu/data", Imu,rcv)
    rospy.Subscriber("pitch", Float64, pitch_rcv)

    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print e
        pass
    finally:

       angle_pitch = Float64()
       angle_roll= Float64()
       angle_yaw= Float64()
       angle_pitch.data=0
       angle_yaw.data=0
       angle_roll.data=0
       yaw.publish(angle_yaw)
       pitch.publish(angle_pitch)
       roll.publish(angle_roll)