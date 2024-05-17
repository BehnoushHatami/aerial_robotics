#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import tf.transformations
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode
import math

aruco_position = PoseStamped() 
aruco_detected = False

def set_drone_mode(mode):
    rospy.wait_for_service('/minihawk_SIM/mavros/set_mode')
    try:
        set_mode = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
        response = set_mode(custom_mode=mode)
        if response.mode_sent:
            rospy.loginfo("Mode set to %s successfully", mode)
        else:
            rospy.loginfo("Failed to set mode to %s", mode)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def detection_callback(data):
    global aruco_position
    global aruco_detected
    if not data.detections:
        aruco_detected = False
    else:
        aruco_detected = True
        for detection in data.detections:
            pose = detection.pose.pose.pose
            aruco_position.pose = pose

def control():
    global aruco_position
    global aruco_detected
    rospy.init_node('apriltag_detector_node', anonymous=True)
    rospy.Subscriber("/minihawk_SIM/MH_usb_camera_link_optical/tag_detections", AprilTagDetectionArray, detection_callback)
    rospy.sleep(1)
    rate=rospy.Rate(30)
    rc_pub = rospy.Publisher("/minihawk_SIM/mavros/rc/override", OverrideRCIn, queue_size=10)
    rospy.sleep(1)
    n_print = 0
    flag_landing = 0
    while not rospy.is_shutdown():
        if  not aruco_detected and flag_landing == 0:
            rospy.loginfo("No AprilTags detected.")
            z_offset = 1600
            x_offset = 1500
            y_offset = 1500
            yaw_offset = 1500
        else:
            ##### stop ####
            x = aruco_position.pose.position.x
            y = aruco_position.pose.position.y
            y += 1
            z = aruco_position.pose.position.z
            quaternion = (aruco_position.pose.orientation.x, aruco_position.pose.orientation.y, aruco_position.pose.orientation.z, aruco_position.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            if yaw < 0 :
                yaw_offset = 1500 + (-math.sqrt(abs(yaw)) * 90)
            else:
                yaw_offset = 1500 + (math.sqrt(abs(yaw)) * 90)
            if n_print > 5:
                rospy.loginfo("Roll: %s, Pitch: %s, Yaw: %s", roll, pitch, yaw)  
                rospy.loginfo("x=%f, y=%f, z=%f", x, y-1, z)
                n_print = 0
            n_print += 1
            if x < 0 :
                x_offset = 1500 + (-math.sqrt(abs(x)) * 50)
            else:
                x_offset = 1500 + (math.sqrt(abs(x)) * 50)
            if y < 0 :
                y_offset = 1500 + (math.sqrt(abs(y)) * 50)
            else:
                y_offset = 1500 + (-math.sqrt(abs(y)) * 50)

            if (abs(x) < 0.2  and abs(y)-1 < 0.2 and abs(yaw) < 0.1):
                rospy.loginfo("Landing . . .")
                z_offset = 1400
                flag_landing = 1
                if z < 7.7:
                    set_drone_mode('QLAND')
                    rospy.sleep(10)
                    break
            else :
                z_offset = 1500
        rc_msg = OverrideRCIn()
        rc_msg.channels =  [x_offset, y_offset, z_offset, yaw_offset, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        rc_pub.publish(rc_msg)
        rate.sleep()

if __name__ == '__main__':
    desired_mode = 'QLOITER'
    set_drone_mode(desired_mode)
    control()
    rospy.spin()