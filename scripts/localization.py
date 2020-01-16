#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from pf_localisation.msg import situation

position = []
have_pose = False
publisher = 0


def callback(data):
    global position
    global have_pose
    position =data.pose.pose #remember postion
    have_pose=True #we have postion




def callback2(data):
    global position
    global have_pose
    
    global publisher
    #rospy.loginfo(have_pose)
    #rospy.loginfo(position)
    if have_pose: #if we have already received position, combine laser and position data and send them
        
        situat = situation()
        situat.header.frame_id = "map"
        situat.pose = position
        
        situat.scan = data
        #rospy.loginfo(situat.scan)
        publisher.publish(situat)


def listener():
    global publisher
    publisher = rospy.Publisher("/estimatedsituation", situation, queue_size=2)
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("base_scan", LaserScan, callback2) #when laser data are received

    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback) #when position data are received

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
