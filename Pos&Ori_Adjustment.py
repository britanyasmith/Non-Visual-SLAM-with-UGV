#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import sqrt
import time
import numpy
import paho.mqtt.client as mqtt 
import time
from random import randrange, uniform


## inputs
target_x = 0.0  # m
target_y = 0.0  # m
target_theta = 0.0  # degree

count = 0
x = 0.0
y = 0.0 
theta = 0.0

received_status = 0



def on_message(client, userdata, message):
    '''This function's primary purpose is to get the message sent
       by the client and parse it so it can be used in the movement 
       of the robot'''

    global target_x
    global target_y
    global target_theta
    global received_status

    print("Received data")
    received_data = str(message.payload.decode("utf-8"))
    print("Data Recieved: ", received_data)
    parsed_string = received_data.split(",")
    target_x = float(parsed_string[0])
    target_y = float(parsed_string[1])
    target_theta = float(parsed_string[2])
    
    received_status = 1


def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()



mqttBroker ="broker.hivemq.com" 
topic = "movement_without_nav" 

client = mqtt.Client("UGV_Lidar")
client.on_message=on_message

client.connect(mqttBroker) 


message = "1"
client.publish(topic, message)
print("Just published: " + message)


client.loop_start()
client.subscribe(topic)
# time.sleep(10) # wait
while received_status == 0:
    time.sleep(0.1)

client.loop_stop() #stop the loop


message = "0"
client.publish(topic, message)
print("Just published: " + message)


while count == 0:
    time.sleep(3)
    #goal.x = x + target_x 

    

    angle_to_goal = atan2(target_y, target_x)
    #if angle_to_goal < 0:
    #    angle_to_goal = angle_to_goal + 6.28319

    goal_theta = target_theta*3.14159/180
    #if target_theta < 0:
    #    goal_theta = goal_theta + 6.28319

    print("angle to goal: ", angle_to_goal)
    print("goal_theta: ", angle_to_goal)

    count = count + 1


dis_to_goal = sqrt(target_x**2 + target_y**2)
prev_time_dis = time.time()
prev_time_ang = time.time()
c = 0

while not rospy.is_shutdown():

    if angle_to_goal > 0 :
        if angle_to_goal - theta > 0.0807:  # tolerance = 0.087 rad = 5 degree
            speed.linear.x = 0.0
            speed.angular.z = 0.1
            pub.publish(speed)
            r.sleep()
            print("remained angle to goal", angle_to_goal - theta)

        elif dis_to_goal - abs(x) > 0.1: # tolerance = 0.1 m
            speed.linear.x = 0.1
            speed.angular.z = 0.0
            pub.publish(speed)
            r.sleep()
            print("remained distance to goal", dis_to_goal - abs(x))

        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            r.sleep()
            break

    elif angle_to_goal < 0 :
        if theta - angle_to_goal > 0.0807:  # tolerance = 0.087 rad = 5 degree
            speed.linear.x = 0.0
            speed.angular.z = -0.1
            pub.publish(speed)
            r.sleep()
            print("remained angle to goal", theta - angle_to_goal)

        elif dis_to_goal - abs(x) > 0.1: # tolerance = 0.1 m
            speed.linear.x = 0.1
            speed.angular.z = 0.0
            pub.publish(speed)
            r.sleep()
            print("remained distance to goal", dis_to_goal - abs(x))

        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            r.sleep()
            break

    else:
        if dis_to_goal - abs(x) > 0.1: # tolerance = 0.1 m
            speed.linear.x = 0.1
            speed.angular.z = 0.0
            pub.publish(speed)
            r.sleep()
            print("remained distance to goal", dis_to_goal - abs(x))

        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            r.sleep()
            break



while(True):
    if (goal_theta > 0 and theta > 0) or (goal_theta < 0 and theta < 0):
        if goal_theta - theta > 0.0807:  # tolerance = 0.087 rad = 5 degree
            speed.linear.x = 0.0
            speed.angular.z = 0.1
            pub.publish(speed)
            r.sleep()
            print("remained angle to target angle", goal_theta - theta)
        elif goal_theta - theta < -0.0807:
            speed.linear.x = 0.0
            speed.angular.z = -0.1
            pub.publish(speed)
            r.sleep()
            print("remained angle to target angle", goal_theta - theta)
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            r.sleep()
            break

    elif (goal_theta < 0 and theta > 0) or (goal_theta > 0 and theta < 0):
        if goal_theta - theta > 0.0807 or goal_theta - theta < -0.0807:  # tolerance = 0.087 rad = 5 degree
            speed.linear.x = 0.0
            speed.angular.z = 0.1
            pub.publish(speed)
            r.sleep()
            print("remained angle to target angle", goal_theta - theta)
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            r.sleep()
            break


message = "1"
client.publish(topic, message)
print("Just published: " + message)