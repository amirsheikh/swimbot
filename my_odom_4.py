#!/usr/bin/env python
import math
from math import sin, cos, pi
import serial
import time
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

global serialport
global data_list
global tmp_value
global data_count
global error_count
global MAX_Speed
global MIN_Speed
global X_pos
global Y_pos
global ang
global V_ang
global V_linear
global last_time

def init():
    global serialport
    global MAX_Speed
    global MIN_Speed
    global data_list
    global tmp_value
    global X_pos
    global Y_pos
    global ang
    global V_ang
    global V_linear
    global last_time


    MAX_Speed = 20
    MIN_Speed = -20

    serialport = serial.Serial ("/dev/ttyS0")
    serialport.baudrate = 115200

    data_list = []
    tmp_value = ""

    X_pos = 0.0
    Y_pos = 0.0
    ang = 0.0
    V_ang = 0
    V_linear = 0

    last_time = rospy.Time.now()
    rospy.Subscriber("/cmd_vel", Twist, callback)


def speed_maker(speed):
    global MAX_Speed
    global MIN_Speed
    if(speed >= MIN_Speed and speed <= MAX_Speed):
        if(speed >= 0):
            digit_10 = int((speed/10)%10)
            digit_1 = int(speed%10)
            digit_01 = int((speed*10)%10)
            result = "+" + str(digit_10) + str(digit_1) + str(digit_01)
        else:
            speed= -1*speed
            digit_10 = int((speed/10)%10)
            digit_1 = int(speed%10)
            digit_01 = int((speed*10)%10)
            result = "-" + str(digit_10) + str(digit_1) + str(digit_01)
            print(result)
    elif(speed < MIN_Speed):
        result = speed_maker(MIN_Speed)
    else:
        result = speed_maker(MAX_Speed)
    return result



def callback(msg):
    global MR_speed
    global ML_speed

    rospy.loginfo("Received a /cmd_vel message!")
    MR_speed = msg.linear.x*100 - msg.angular.z * 12.75
    ML_speed = msg.linear.x*100 + msg.angular.z * 12.75

    ML_speed = speed_maker(ML_speed)
    MR_speed = speed_maker(MR_speed)
    data = MR_speed + ML_speed
    serialport.write(data.encode())

    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))


def data_handler(V_linear,V_ang):
    global X_pos
    global Y_pos
    global ang
    global last_time

    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    delta_x = (V_linear * cos(ang)) * dt
    delta_y = (V_linear * sin(ang)) * dt
    delta_th = V_ang * dt

    X_pos += delta_x
    Y_pos += delta_y
    ang += delta_th

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, ang)
    odom_broadcaster.sendTransform(
        (X_pos/100.0, Y_pos/100.0, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(X_pos/100.0, Y_pos/100.0, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(V_linear/100.0, 0, 0), Vector3(0, 0, V_ang))
    odom_pub.publish(odom)

    last_time = current_time



init()
rospy.init_node('SWIM_BOT')
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

while not rospy.is_shutdown():
    try:
        read_byte = serialport.read()
        if(read_byte.decode() == '\n'):
            data_list.append(float(tmp_value))
            tmp_value = ""
            V_ang=(data_list[1] - data_list[0])/25.5
            V_linear=(data_list[1] + data_list[0])/2
            data_handler(V_ang,V_linear)
            data_list = []
        elif(read_byte.decode() == ' '):
            data_list.append(float(tmp_value))
            tmp_value = ""
        else:
            tmp_value = tmp_value + (read_byte.decode())           
    except Exception as e:
        global error_count
        error_count = error_count +1
        rospy.loginfo("ERROR", error_count)
        print(error_count,e)
        init()

