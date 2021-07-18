#!/usr/bin/env python

import math
from math import sin, cos, pi
import serial
import time
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
global Xi
Xi = 0
global Yi
Yi = 0
def callback(msg):
    global Xi
    global Yi
    rospy.loginfo("Received a /cmd_vel message!")
    #Xi=msg.linear.x #Vx   xi = y - Wz * 12.75
    #Yi=msg.linear.y #Vy
    Xi=msg.linear.y - msg.angular.z * 12.75
    Yi=msg.linear.y + msg.angular.z * 12.75
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

vx = 0
vy = 0
vth = 0

current_time = rospy.Time.now()
last_time = rospy.Time.now()
ser = serial.Serial ("/dev/ttyS0",timeout=10000)    #Open named port 
ser.baudrate = 115200                     #Set baud rate to 9600  
flag = 1
S=""
Velocities=[]
r = rospy.Rate(1000.0)
rospy.Subscriber("/cmd_vel", Twist, callback)
while not rospy.is_shutdown():
    
    if(flag):
        Vels=[]
        
        Vels.append(Xi)
        
        Vels.append(Yi)
        OutVel=[]
        for i in Vels:
            if(abs(float(i))>=10):
                A=int(round(float(i),1)*10)
                if(A<0):
                    Out1="-"+str(abs(A))
                else:
                    Out1="+"+str(A)
                OutVel.append(Out1)
            elif(abs(float(i))<10):
                A=int(round(float(i),1)*10)
                if(abs(float(i))<1):
                    Out2="0"+str(abs(A))
                else:
                    Out2=str(abs(A))
                
                if(A<0):
                    Out2="-0"+Out2
                else:
                    Out2="+0"+Out2
                OutVel.append(Out2)
        OutVelF=OutVel[0]+OutVel[1]
        #print(OutVel[0])
        #print(OutVel[1])
        OutVel = []
        
        ser.write(OutVelF.encode())
        #print(OutVelF)
        #print("error1")
        flag = 0
    data = ser.read()                      #Send back the received data
    
    S=S+data.decode()
    if(data.decode()==" "):
        Velocities.append(float(S))
        S=""
    elif(data.decode()=='\n'):
        Velocities.append(float(S))
        S=""
        vth=(Velocities[1]-Velocities[0])/25.5
        vy=(Velocities[1]+Velocities[0])/2
        #print(vy)
        th=Velocities[2]
        #rospy.loginfo(th)
        #print(Velocities)
        Velocities = []
        flag = 1
    
        

    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    #delta_th = vth * dt

    x += delta_x
    y += delta_y
    #th += delta_th
 
    
    

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()

