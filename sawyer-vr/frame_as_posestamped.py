#!/usr/bin/env python
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import rospy
import tf
import sys
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
import numpy as np
import triad_openvr
from htc_vive_teleop_stuff.msg import itzy
import pylab
import csv
import os
import pickle
from nav_msgs.msg import Odometry
import tf_conversions.posemath as pm
"""
Publish a frame 3D pose as a PoseStamped continuously.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


# CUSTOM MSG WILL WORK LIKE THESE
# yuna = itzy()
# yuna.jyp = [r,p,y]
# yuna.ymg.position.x = ~~
# yuna.ymg.position.y = ~~
# yuna.ymg.position.z = ~~
#
# data type of topic msg --> will be 'itzy'
# what we actually have to publish is 'yuna' 


# try:
#     ###################
#     # Initialize OpenVR
#     v = triad_openvr.triad_openvr()

# except Exception as ex:
#     if (type(ex).__name__ == 'OpenVRError' and ex.args[0] == 'VRInitError_Init_HmdNotFoundPresenceFailed (error number 126)'):
#         print('Cannot find the tracker.')
#         print('Is SteamVR running?')
#         print('Is the Vive Tracker turned on, connected, and paired with SteamVR?')
#         print('Are the Lighthouse Base Stations powered and in view of the Tracker?\n\n')
#     else:
#         template = "An exception of type {0} occurred. Arguments:\n{1!r}"
#         message = template.format(type(ex).__name__, ex.args)



# P = np.mat([[1e-6, 0, 0], [0, 1e-6, 0], [0, 0, 1e-3]])
# p_cov = np.zeros((6, 6))

# # position covariance
# p_cov[0:2,0:2] = P[0:2,0:2]

# # orientation covariance for Yaw
# # x and Yaw
# p_cov[5,0] = p_cov[0,5] = P[2,0]

# # y and Yaw
# p_cov[5,1] = p_cov[1,5] = P[2,1]

# # Yaw and Yaw
# p_cov[5,5] = P[2,2]

# p_cov[0,:] =   [0.0000349162103240595,  -0.0000018202960310455,  -0.0000339898160507969,  -0.0000081126791170800,   0.0000001353045808767,   0.0000032202291901186]
# p_cov[1,:] =    [-0.0000018202960310455,   0.0000011910722363973,   0.0000020423436706964,   0.0000010961526869235,  -0.0000000333091396801,  -0.0000001408541892558]
# p_cov[2,:] =    [-0.0000339898160507969,   0.0000020423436706964,   0.0000341312090595451,   0.0000060715616751347,  -0.0000000237628610568,  -0.0000029217229365340]
# p_cov[3,:] =    [-0.0000081126791170800,   0.0000010961526869235,   0.0000060715616751347,   0.0000165832615351042,  -0.0000004759697840205,  -0.0000024486872043021]
# p_cov[4,:] =    [0.0000001353045808767,  -0.0000000333091396801,  -0.0000000237628610568,  -0.0000004759697840205,   0.0000003366392930324,  -0.0000000030521109214]
# p_cov[5,:] =    [0.0000032202291901186,  -0.0000001408541892558,  -0.0000029217229365340,  -0.0000024486872043021,  -0.0000000030521109214,   0.0000007445433570531]


class PublishFrameAsPoseStamped(object):
    def __init__(self, frame_to_posestamped,
                 reference_frame,
                 rate,
                 verbose=False):
        """
        Class to publish a frame as a PoseStamped.
        :param frame_to_posestamped str: frame that will be published its
                pose as PoseStamped.
        :param reference_frame str: frame that will be the header.frame_id
                of the PoseStamped.
        :param rate int: rate at which to compute and publish the pose.
        :param verbose bool: print to screen the transformations.
        """
        self.tf_l = tf.TransformListener()
        topic_name = frame_to_posestamped.replace('/', '')
        self.pose_pub = rospy.Publisher(topic_name + '_as_posestamped', # --> I think it will be Controller
                                        PoseStamped, queue_size=10)
        
        self.frame_to_posestamped = frame_to_posestamped                # --> I think it will be Controller
        
        self.rrrate = rate
        self.reference_frame = reference_frame                          # --> I think it will be Headset
        self.rate = rospy.Rate(rate)
        self.verbose = verbose
        #self.delta_pub = rospy.Publisher("test", Pose, queue_size=10)
        
        
        self.delta_pub = rospy.Publisher("test", itzy, queue_size=10)    # FOR JOINT POSITION COMMAND
        
        
        self.yuna = itzy()   # custom msg
        
        
        self.Jvel_pub = rospy.Publisher("Jvel", Odometry, queue_size=50)
    


    def transform_pose(self, pose, from_frame, to_frame):
        """
        Transform the 'pose' from frame 'from_frame'
         to frame 'to_frame'

        :param geometry_msgs/Pose pose: 3D Pose to transform.
        :param str from_frame: frame that the pose belongs to.
        :param str to_frame: to what frame transform.
        """
        ps = PoseStamped()
        # ps.header.stamp = #self.tf_l.getLatestCommonTime(from_frame,
        # to_frame)
        ps.header.frame_id = from_frame   # identify the Controller_id
        
        ps.pose = pose                    # pose --> will be assigned instance of 'Pose()'
        
        transform_ok = False
        min_time_in_between_warns = rospy.Duration(5.0) # originally it was --> 'rospy.Duration(5.0)'

        print(rospy.Duration(5.0))
        print(min_time_in_between_warns)
        
        last_warn = rospy.Time.now() #- min_time_in_between_warns
        while not transform_ok and not rospy.is_shutdown():
            try:
                target_ps = self.tf_l.transformPose(to_frame, ps) # this will be returned
                
                transform_ok = True
            
            except tf.ExtrapolationException as e:
                if rospy.Time.now() > (last_warn + min_time_in_between_warns):
                    rospy.logwarn(
                        "Exception on transforming pose... trying again \n(" +
                        str(e) + ")")
                    last_warn = rospy.Time.now()
                rospy.sleep(0.2)
                ps.header.stamp = self.tf_l.getLatestCommonTime(
                    from_frame, to_frame)
            except tf.LookupException as e:
                if rospy.Time.now() > (last_warn + min_time_in_between_warns):
                    rospy.logwarn(
                        "Exception on transforming pose... trying again \n(" +
                        str(e) + ")")
                    last_warn = rospy.Time.now()
                rospy.sleep(1.0)

        target_ps.header.stamp = rospy.Time.now()
        return target_ps

    def run(self):
        ps = Pose()
        ps.orientation.w = 1.0  # Quaternion must be correct
        
        # X=[]
        # R = []
        # Y = []
        # k=0
        prex, prey, prez, preqx, preqy, preqz, preqw = 0,0,0,0,0,0,0
        #prex, prey, prez, prer, prep, preyy = 0,0,0,0,0,0
        while not rospy.is_shutdown():
            tfed_ps = self.transform_pose(ps,
                                          self.frame_to_posestamped, # --> I think it will be Controller
                                          self.reference_frame)      # --> I think it will be Headset
            self.yuna.ymg = Pose(Point(prex, prey, prez), Quaternion(preqx, preqy, preqz, preqw))
            self.yuna.ymg_2 = Pose(Point(tfed_ps.pose.position.x, tfed_ps.pose.position.y, tfed_ps.pose.position.z), Quaternion(tfed_ps.pose.orientation.x, tfed_ps.pose.orientation.y, tfed_ps.pose.orientation.z, tfed_ps.pose.orientation.w))

            [prex, prey, prez, preqx, preqy, preqz, preqw] = [tfed_ps.pose.position.x, tfed_ps.pose.position.y, tfed_ps.pose.position.z, tfed_ps.pose.orientation.x, tfed_ps.pose.orientation.y, tfed_ps.pose.orientation.z, tfed_ps.pose.orientation.w]

            self.delta_pub.publish(self.yuna)
            self.rate.sleep()






if __name__ == '__main__':
    rospy.init_node('frame_to_posestamped')
    argv = rospy.myargv(sys.argv)
    if len(argv) < 3:
        print("Usage:")
        print(argv[0] + " frame_to_posestamped reference_frame [rate]")
        exit(0)
    
    frame_to_posestamped = argv[1]   # --> I think it will be Controller

    #print(argv[1])
    
    reference_frame = argv[2]        # --> I think it will be Headset
    #print(argv[2])
    print(reference_frame)
    print(1111111111111111111111111111111111111111111)
    if len(argv) == 4:
        rate = int(argv[3])
    else:
        rate = 10
    pfaps = PublishFrameAsPoseStamped(frame_to_posestamped,  # --> I think it will be Controller
                                      reference_frame,       # --> I think it will be Headset
                                      rate=100,
                                      verbose=True)
    pfaps.run()
