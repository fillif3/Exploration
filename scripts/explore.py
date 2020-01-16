#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

left = 1 #if obstalce is on the left
front = 0 #if obstalce is on the front
state = 4 #state (first state is 4th state)
times=0


def where_obstacle(lasers_data): # Check where are obstacles
    global left
    global front
    ranges = lasers_data.ranges
    # We have 6 groups of lasers
    suml = [0, 0,0,0,0,0] # summ of all groups
    j = [0, 0,0,0,0,0] # amount of lasers which were used 
    border_laser = (660, 700, 320, 350, 350, 410, 410, 475, 335 , 380, 380, 440) # 6 groups - each grous has first laser in group and last, each pair is for one group
    # First group is for detecting obstacle on the left side, rest of the is on the front.
    for k in range(len(border_laser) / 2): #for each pair
        for l in range(border_laser[2 * k + 1] - border_laser[2 * k] + 1): #for each laser in the group
            m = border_laser[2 * k] + l #current laser in the group
	        mesur = ranges[m] # laser data
            if (math.isnan(mesur) == False) and (0.02 < mesur < 5.6): #If range is good (number and not too low or high)
                suml[k] = suml[k] + mesur #sum all lasers data
                j[k] = j[k] + 1 #more lasers were used
        if j[k] > 0: # send iformation of each group
            rospy.loginfo('The %d th mesur of %d lasers is %f', k, j[k], (suml[k] / j[k]))
    # Adding histeresis for detecting obstacles
    if left:
        a = 0.1
    else:
        a = -0.1
    if front:
        b = 0.1
    else:
        b = -0.1
    # Check if 'front' groups dected obstacle 
    if j[1] == 0:
        fronta = 0
    elif (suml[1] / j[1]) > (1 + b):
        fronta = 0
    else:
        fronta = 1
    if j[2] == 0:
        frontb = 0
    elif (suml[2] / j[2]) > (1 + b):
        frontb = 0
    else:
        frontb = 1
    if j[3] == 0:
        frontc = 0
    elif (suml[3] / j[3]) > (1 + b):
        frontc = 0
    else:
        frontc = 1
    if j[4] == 0:
        frontd = 0
    elif (suml[4] / j[4]) > (1 + b):
        frontd = 0
    else:
        frontd = 1
    if j[5] == 0:
        fronte = 0
    elif (suml[5] / j[5]) > (1 + b):
        fronte = 0
    else:
        fronte = 1
    # if any of them did, there is obstacle on the robot's front
    if fronta or frontb or frontc or frontd or fronte:
        front = 1
    else:
        front=0
    # same but for left side
    if j[0] == 0:
        left = 0
    elif (suml[0] / j[0]) > (1.2 + a):

        left = 0
    else:
        left = 1
    

def where_go(): #Depending on obstacles, robot decides where it should go (turn left, move forward or turn right) 
    global left
    global front
    global state
    global times #Used when we are in state 2 (turn left). If after 10 iterations robot did not find wall, it probably disappeard (maybe it was dog, not wall etc.)  
    # states - follow left wall, turn left, turn right, move forward (wall disappeard)
    # 
    if state == 1: 
        times = 0
        if left == 0 and front == 0:
            state = 2
        elif front == 1:
            state = 3
    elif state == 2:
	times=times+1
        if left == 1 and front == 0:
            state = 1
        elif times == 10:
           state = 4
    elif state == 3:
        times = 0
        if left == 1 and front == 0:
            state = 1
        elif left == 0 and front == 0:
            state = 4
    elif state == 4:
        times = 0
        if left == 1 and front == 0:
            state = 1
        elif front == 1:
            state = 3
    rospy.loginfo('The state is %d', state) #send state

def choose_cmd_vel(): #depending on state, compute velocity
    global state
    if state == 1 or state == 4:
        v = 0.1
        w = 0
    elif state == 2:
        v = 0
        w = 1
    elif state == 3:
        v = 0
        w = -1
    rospy.loginfo('''linear is %f
angular is %f''', v,w) #Send velocity to terminal
    return v, w



def talker(v, w): #Send velocity to motors
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    if not rospy.is_shutdown():
        base_data = Twist()
        base_data.linear.x = v
        base_data.angular.z = w
        pub.publish(base_data)


def callback(data):
    where_obstacle(data)
    where_go()
    v, w = choose_cmd_vel()
    talker(v,w)
    rospy.loginfo('Left - %d and front - %d', left, front)


# if __name__ == '__main__':
#		try:
#			talker(a/4,max(b/4-0.025,0))
#		except rospy.ROSInterruptException:
#			pass

def listener(left, front):
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('base_scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener(left, front)
