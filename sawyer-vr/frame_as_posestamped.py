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
            # We transform a pose with reference frame
            # self.frame_to_posestamped
            # which is 0.0, 0.0, 0.0
            # to the reference frame to get it's pose
            
            tfed_ps = self.transform_pose(ps,
                                          self.frame_to_posestamped, # --> I think it will be Controller
                                          self.reference_frame)      # --> I think it will be Headset
            
            

            
            # tfed_ps.pose.position.x    --> this CODE will work I think, considering the mechanics of the work flow
            # ~~
            # tfed_ps.pose.orientation.x --> this CODE will work I think, considering the mechanics of the work flow
            # ~~
                    

            #r, p, y = euler_from_quaternion([tfed_ps.pose.orientation.x, tfed_ps.pose.orientation.y, tfed_ps.pose.orientation.z, tfed_ps.pose.orientation.w])
            
            #delta = np.array([tfed_ps.pose.position.x, tfed_ps.pose.position.y, tfed_ps.pose.position.z, tfed_ps.pose.orientation.x, tfed_ps.pose.orientation.y, tfed_ps.pose.orientation.z, tfed_ps.pose.orientation.w]) - np.array([prex, prey, prez, preqx, preqy, preqz, preqw])
            
            #delta = np.array([tfed_ps.pose.position.x, tfed_ps.pose.position.y, tfed_ps.pose.position.z, r, p, y]) - np.array([prex, prey, prez, prer, prep, preyy])
            
            #print([deltax, deltay, deltaz, deltaqx, deltaqy, deltaqz, deltaqw])
            #pose_mg = Pose(Point(delta[0], delta[1], delta[2]), Quaternion(delta[3],delta[4],delta[5],delta[6])) 
            
            # R.append(delta[3])
            # Y.append(delta[5])
            # X.append(k)
            #quaternion_from_euler(math.pi, 0.0, -math.pi/2.0) 
            




            self.yuna.ymg   = Pose(Point(prex, prey, prez), Quaternion(preqx, preqy, preqz, preqw))
            
            self.rate.sleep()
            
            self.yuna.ymg_2 = Pose(Point(tfed_ps.pose.position.x, tfed_ps.pose.position.y, tfed_ps.pose.position.z), Quaternion(tfed_ps.pose.orientation.x, tfed_ps.pose.orientation.y, tfed_ps.pose.orientation.z, tfed_ps.pose.orientation.w))

 
            # #FOR JOINT POSITION COMMAND
            # #JUST dx ('x' here means Cartesian)
            # self.yuna.jyp = [prer, prep, preyy]  # r_t, p_t, y_t 
            # self.yuna.ymg.position.x = prex      # x_t
            # self.yuna.ymg.position.y = prey      # y_t
            # self.yuna.ymg.position.z = prez      # z_t
            
            # self.yuna.jyp_2 = [r, p, y]                            # r_t+1, p_t+1, y_t+1 
            # self.yuna.ymg_2.position.x = tfed_ps.pose.position.x   # x_t+1
            # self.yuna.ymg_2.position.y = tfed_ps.pose.position.y   # y_t+1
            # self.yuna.ymg_2.position.z = tfed_ps.pose.position.z   # z_t+1
           

            # #FOR JOINT POSITION COMMAND
            # #JUST dx ('x' here means Cartesian)
            # self.yuna.jyp = [delta[3], delta[4], delta[5]]  # delta r, delta p, delta y 
            # self.yuna.ymg.position.x = delta[0]             # delta x
            # self.yuna.ymg.position.y = delta[1]             # delta y
            # self.yuna.ymg.position.z = delta[2]             # delta z
            

            # # FOR JOINT VELOCITY COMMAND
            # # dx/dt ('x' here means Cartesian)
            # self.yuna.jyp = [delta[3]/self.rrrate, delta[4]/self.rrrate, delta[5]/self.rrrate]   # dr/dt, dp/dt, dy/dt
            # self.yuna.ymg.position.x = delta[0]/self.rrrate                                      # dx/dt
            # self.yuna.ymg.position.y = delta[1]/self.rrrate                                      # dy/dt
            # self.yuna.ymg.position.z = delta[2]/self.rrrate                                      # dz/dt
            

                                                                                                 # dr/dt, dp/dt, dy/dt
            #self.yuna.yeji = Twist(Vector3(vx, vy, vz), Vector3(v_roll, v_pitch, v_yaw))         # dx/dt
           





            #pose_mg = Pose(Point(x, y, z), Quaternion(qx, qy, qz, qw))
            #delta_pub = rospy.Publisher("test", Pose, queue_size=1)
            self.delta_pub.publish(self.yuna)
            



            [prex, prey, prez, preqx, preqy, preqz, preqw] = [tfed_ps.pose.position.x, tfed_ps.pose.position.y, tfed_ps.pose.position.z, tfed_ps.pose.orientation.x, tfed_ps.pose.orientation.y, tfed_ps.pose.orientation.z, tfed_ps.pose.orientation.w]
            #[prex, prey, prez, prer, prep, preyy] = [tfed_ps.pose.position.x, tfed_ps.pose.position.y, tfed_ps.pose.position.z, r, p, y]

            
            # pose = np.array([tfed_ps.pose.position.x, tfed_ps.pose.position.y, tfed_ps.pose.position.z, tfed_ps.pose.orientation.x, tfed_ps.pose.orientation.y, tfed_ps.pose.orientation.z, tfed_ps.pose.orientation.w])
            # pose_mg = Pose(Point(pose[0], pose[1], pose[2]), Quaternion(pose[3], pose[4], pose[5],pose[6]))
            # delta_pub = rospy.Publisher("pose", Pose, queue_size=1)
            # delta_pub.publish(pose_mg)
            
            
            # print(tfed_ps)
            #self.pose_pub.publish(tfed_ps)
            
            # if self.verbose:
            #     print(tfed_ps)
            
            #self.rate.sleep()

            # if rospy.is_shutdown():
            # os.chdir('/home/irobot/Desktop')
            # X=[]
            # with open ('roll.bin', 'wb') as f_1:             
            #     pickle.dump(R, f_1)
            # with open ('yaw.bin', 'wb') as f_2:             
            #     pickle.dump(Y, f_2)    
            # X=[]
            # for k in range(len(R)):
            #     X.append(k)

            # os.chdir('/home/irobot/Desktop')
            # f = open('roll.csv', 'wb')             
            # wr = csv.writer(f)
            # for row in D:
            #     wr.writerow(row)
            # #wr.writerow([2, 'co'])
            # #wr.writerow([3, 'kr'])
            # f.close()pylab.plot(episodes, scores, 'b')
            
            
            # pylab.plot(X, R, 'b')
            # pylab.plot(X, Y, 'c')
            # os.chdir('/home/irobot/Desktop')
            # pylab.savefig("YMG_roll.png")

            # k += 1

            #self.rate.sleep()








            # #time = rospy.Time.now() # present time
            # # This represents an estimate of a position and velocity in free space
            # odom = Odometry()     # make class Odometry() as a instance
            #                     # 'sensor' which measures 'position' & 'rotation'
        
            # #odom.header.stamp = time
            # #odom.header.frame_id = "vive_world"
            # odom.pose.pose = Pose(Point(tfed_ps.pose.position.x, tfed_ps.pose.position.y, tfed_ps.pose.position.z), Quaternion(tfed_ps.pose.orientation.x, tfed_ps.pose.orientation.y, tfed_ps.pose.orientation.z, tfed_ps.pose.orientation.w))  # make class Pose() as a instance
            # #odom.child_frame_id = "ak1_base_link"
            # [vx, vy, vz, v_roll, v_pitch, v_yaw] = v.devices['controller_2'].get_velocities()
            # odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(v_roll, v_pitch, v_yaw))
            
            # # This is all wrong but close enough for now
            # # np.mat([1[1e-6, 0, 0], [0, 1e-6, 0], [0, 0, 1e-3]])
            # #odom.pose.covariance = tuple(p_cov.ravel().tolist())
            # #odom.twist.covariance = tuple(p_cov.ravel().tolist())
            # self.Jvel_pub.publish(odom)





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
