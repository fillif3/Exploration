#!/usr/bin/env python
import numpy as np
import math
import random

import rospy

from pf_localisation.msg import situation
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped,PoseArray, Quaternion, Twist,Pose
from nav_msgs.msg import OccupancyGrid, Path 
from util import rotateQuaternion, getHeading
from std_msgs.msg import String

data = []
small_map = []

pub = 0
pub2= 1

if_returning = False

def navigate(estimated_pose, next_point, scan, kp, kn, maximum_velocity, positive_border, negative_border):
    # Finds best velocity for 1. moving through created path 2. Avoiding obsacles

    dif_x = next_point.position.x - estimated_pose.position.x
    dif_y = next_point.position.y - estimated_pose.position.y
    #where_obstacle(scan)
    # differences between current and next position
    forcepx, forcepy = positive_force(dif_x, dif_y, kp, positive_border) #compute positive foces usuing parameters
    rospy.loginfo('positive')
    rospy.loginfo(forcepx)
    rospy.loginfo(forcepy)
    forcepx, forcepy = rotation(forcepx, forcepy, getHeading(estimated_pose.orientation)) #Rotate forces to robot coordinate axis from map axis
    rospy.loginfo('rotation')
    rospy.loginfo(forcepx)
    rospy.loginfo(forcepy)
    rospy.loginfo(getHeading(estimated_pose.orientation))
    # following the path
    forcenx, forceny = negative_force(kn, scan, negative_border, 0) #Compute negative forces
    rospy.loginfo('negative')
    rospy.loginfo(forcenx)
    rospy.loginfo(forceny)
    # avoiding obstacles
    #rospy.loginfo('sum')
    #forcex = forcepx + forcenx
    #forcey = forcepy + forceny
    #rospy.loginfo(forcex)
    #rospy.loginfo(forcey)
    #vx, vy = compute_velocity(forcex, forcey, maximum_velocity)
    force_robot_x = forcepx + forcenx #add them
    force_robot_y = forcepy + forceny
#rotation(forcex, forcey, getHeading(estimated_pose.orientation))
    
    rospy.loginfo('final forces')
    rospy.loginfo(force_robot_x)
    rospy.loginfo(force_robot_y)
    rospy.loginfo('speed')
    v,w = controller(force_robot_x,force_robot_y,maximum_velocity,0.2,0.4) #Conroller compute best velocity
    return w,v


def controller(x,y,max_v,pv,pw): #compute best veolcity, basing on inforamtion from artificial potential fields
    distance = distan(0,0,x,y) #compute distance
    global front
    #theta = math.atan2(y,x)
    v = x/distance*pv #compute linear velocity
    w=  y/distance*pw #compute angular velocity
    rospy.loginfo(v)
    rospy.loginfo(w)
    if (x < 0):# if linear velocity is lower than 0
        v=0 #velocity is 0
        w =  math.copysign(pw, w)  #angular veloicty is pw
    if v>max_v: #if linear velocity is too big
        w = w*(max_v/v) #angular velocity is proportionally lower
        v=max_v #linear velocity is equal to max_v
    return v,w


def rotation(vx, vy, yaw): #Rotate vector about chosen angle
    v = vx * math.cos(yaw) + vy * math.sin(yaw)
    w = - vx * math.sin(yaw) + vy * math.cos(yaw)
    return v,w


def distan(x1, y1, x2, y2): #Compute distance between points
    dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return dist


def positive_force(dif_x, dif_y, kp, positive_border): #Compute postive force
    dist = distan(0, 0, dif_x, dif_y)
    if dist > positive_border:
        force = positive_border * kp
    else:
        force = dist * kp
    fx = dif_x / dist * force
    fy = dif_y / dist * force
    return fx, fy


def negative_force(kn, scan, negative_border, yaw):  #Compute negative force
    forcex = 0
    forcey = 0
    angle = scan.angle_min + yaw + 10*scan.angle_increment #compute angle of first group
    amount_of_lasers = len(scan.ranges)
    amount_of_groups = int(amount_of_lasers / 10) #we ahve ten times less groups than lasers
    for i in range(amount_of_groups):
        dist = 0
        k=0
        for j in range(20): #each group has 20 lasers
            if i * 10 + j < amount_of_lasers: #condtion for last group
                if not math.isnan(scan.ranges[i * 10 + j]): #some data from lasers are NaN
                    if scan.range_max > scan.ranges[i * 10 + j] > scan.range_min: #Range must between min and max range
                        dist = dist + scan.ranges[i * 10 + j] #add laser data
                        k=k+1
        if dist > 0:
           # if dist < ((negative_border * 20)+1.5): #f obstacle is close
            if dist < ((negative_border * k)+0.075*k):
               # dist = dist / 20 #divide sum
                dist = dist/k
                if dist>0.075: # robot is not a point so we need to substract a value if scan is lower, we set value to low number
                    dist = dist-0.075
                else:
                    dist = 0.0001 
                force = kn / dist ** 2 * (1 / negative_border - 1 / dist)
                forcex = forcex + force * math.cos(angle)
                forcey = forcey + force * math.sin(angle)

        angle = angle + 10 * scan.angle_increment #increament angle for next group

    return forcex, forcey


def is_small_angle(a1, a2, border): #check if angle is smal (return True or False)
    diff = a2 - a1
    while (diff > math.pi):
        diff = diff - math.pi * 2
    while (diff < -math.pi):
        diff = diff + math.pi * 2
    if (diff < border) and (diff > -border):
        return True
    else:
        return False


def rotate(trajectory): #Try to rotate a robot to go trhough trajectory
    x1 = trajectory.poses[0].pose.position.x
    y1 = trajectory.poses[0].pose.position.y
    x2 = trajectory.poses[1].pose.position.x
    y2 = trajectory.poses[1].pose.position.y
    yaw = math.atan2(y2 - y1, x2 - x1) #compute yaw
    rospy.loginfo(yaw)

    while True:
        try:
            rospy.loginfo('I am here')
            data = rospy.wait_for_message("/estimatedsituation", situation)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                         " running: rosrun map_server map_server <mapname> ")
        rospy.loginfo(getHeading(data.pose.orientation))
        rospy.loginfo(yaw)
        if is_small_angle(yaw, getHeading(data.pose.orientation), 0.2):
            talker(0, 0)
            break
        talker(0.3, 0) #if we are not close, we turn left

def rotate2(yaw): #same as rotate but we have defined yaw
    rospy.loginfo(yaw)

    while True:
        try:
            data = rospy.wait_for_message("/estimatedsituation", situation, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                         " running: rosrun map_server map_server <mapname> ")
        rospy.loginfo(getHeading(data.pose.orientation))
        rospy.loginfo(yaw)
        if is_small_angle(yaw, getHeading(data.pose.orientation), 0.2):
            talker(0, 0)
            break
        talker(0.3, 0)




def talker(w, v):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    base_data = Twist()
    base_data.linear.x = v
    base_data.angular.z = w

    pub.publish(base_data)


def talker_respond_speech(): #sends info to speech recognition

    # rospy.init_node('Mover', anonymous=True)
    pub2.publish('q')


def talker_respond_face():#sends info to face recognition

    # rospy.init_node('Mover', anonymous=True)
    pub.publish('q')



def callback(data2):
    global data
    global small_map
    global if_returning
    rospy.loginfo('We have a trajectory')
    # Finds best velocity for 1. moving through created path 2. Avoiding obsacles
    rotate(data2) #rotate to the best postion to start trajectory
    go_along_trajectory(data2) #go along trajectory

    talker(0, 0)
    if if_returning:
        rotate2(math.pi/2)
        talker_respond_face()
        if_returning=False
    else:
        talker_respond_speech()
        if_returning=True



def go_along_trajectory(trajectory):
    pub = rospy.Publisher('nextpose', PoseStamped, queue_size=100)
    try:
        data = rospy.wait_for_message("/estimatedsituation", situation) #get point from cosen topic
    except:
        rospy.logerr("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> ")
    for point in trajectory.poses:
        k = PoseStamped()
        k.pose = point.pose
        k.header.frame_id = "map"
        pub.publish(k) #publish current position
        while True:
            w, v = navigate(data.pose, point.pose, data.scan, 1, 0.005, 0.2, 1, 0.5) #find best velocity
            talker(w, v)
            try:
                data = rospy.wait_for_message("/estimatedsituation", situation, 20) #get point from cosen topic
            except:
                rospy.logerr("Problem getting a map. Check that you have a map_server"
                             " running: rosrun map_server map_server <mapname> ")
            if is_close(data.pose.position, point.pose.position, 2): #if we are close, break loop
                break
    last_p = trajectory.poses[-1].pose
    while True:
            pub.publish(k)
            w, v = navigate(data.pose, last_p, data.scan, 2, 0.005, 0.1, 1, 0.2) #now we try to go to the last point (rest is the same as previously)
            talker(w, v)
            try:
                data = rospy.wait_for_message("/estimatedsituation", situation, 20)
            except:
                rospy.logerr("Problem getting a map. Check that you have a map_server"
                             " running: rosrun map_server map_server <mapname> ")
            if is_close(data.pose.position, last_p.position, 0.4):
                break


def is_close(point, point2, distance): #if both points are closer to each other than chosen distance, returns True, False otherwise
    dist = distan(point.x, point.y, point2.x, point2.y)
    rospy.loginfo('distance')
    rospy.loginfo(dist)
    if dist < distance:
        return True
    else:
        return False


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global pub
    global pub2
    rospy.init_node('listener', anonymous=True)

    pub = rospy.Publisher('finish', String, queue_size=100)
    pub2 = rospy.Publisher('finished', String, queue_size=100)
    #global data
    #global small_map
    #try:
    #    data = rospy.wait_for_message("/map", OccupancyGrid, 20)
    #except:
    #    rospy.logerr("Problem getting a map. Check that you have a map_server"
     #                " running: rosrun map_server map_server <mapname> ")
    #    sys.exit(1)
    #new_map = create_a_map(data.info.width, data.info.height, data.data)
    #xmin, ymin, xmax, ymax = small_index(new_map, data.info.width, data.info.height)
    #small_map = dilation(new_map, xmin, ymin, xmax, ymax, 0.3, data.info.resolution)
    rospy.loginfo('We have a map')
    rospy.Subscriber('trajectory', Path, callback)
    rospy.loginfo('321321')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()