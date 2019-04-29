#!/usr/bin/env python

# Python Imports
import numpy as np
from math import pow, pi, sqrt
import tf.transformations as tr
import threading
import tf



# ROS Imports
import rospy
from std_msgs.msg import Bool, Int32, Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import intera_interface
from intera_core_msgs.msg import EndpointState

# Local Imports
import sawyer_MR_description as s
import modern_robotics as r
import custom_logging as cl
import time

from gazebo_msgs.srv import (
    SetModelState,
    GetModelState,
    SpawnModel,
    DeleteModel,
)
import PyKDL
from PyKDL import *
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
# conversiom btwn PyKDL frame, ROS message and tf.transformation
import tf_conversions.posemath as pm

from htc_vive_teleop_stuff.msg import itzy

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped, PoseStamped
import time
from numpy.linalg import inv
####################
# GLOBAL VARIABLES #
####################
TIME_LIMIT = 7    #15s
DAMPING = 0.02 # mimimize
JOINT_VEL_LIMIT = 1.5    #2rad/s

CUBIC = 3
QUINTIC = 5

starting_joint_angles = {'right_j0': -0.041662954890248294,
                         'right_j1': -1.0258291091425074,
                         'right_j2': 0.0293680414401436,
                         'right_j3': 1.37518162913313,
                         'right_j4':  -0.06703022873354225,
                         'right_j5': 0.7968371433926965,
                         'right_j6': 1.7659649178699421}

overhead_orientation = Quaternion(
                         x=-0.00142460053167,
                         y=0.999994209902,
                         z=-0.00177030764765,
                         w=0.00253311793936)

class VelocityControl(object):
    def __init__(self):
        rospy.loginfo("Creating VelocityController class")

        # Create KDL model
        with cl.suppress_stdout_stderr():    # Eliminates urdf tag warnings
            self.robot = URDF.from_parameter_server() # sawyer's URDF

        # self.kin = KDLKinematics(self.robot, "base", "right_gripper") # kinematics to custom joint frame
        self.kin = KDLKinematics(self.robot, "base", "right_gripper_tip")

        self.names = self.kin.get_joint_names()

        if rospy.has_param('vel_calc'):
            rospy.delete_param('vel_calc')
        
        
        self.limb = intera_interface.Limb("right")
        #self._limb = Limb()


        self.gripper = intera_interface.gripper.Gripper('right')
        # Grab M0 and Blist from saywer_MR_description.py
        self.M0 = s.M #Zero config of right_hand
        self.Blist = s.Blist #6x7 screw axes mx of right arm
        self.Kp = 50.0*np.eye(6)
        self.Ki = 0.0*np.eye(6)
        self.Kd = 100.0*np.eye(6) # add D-gain
        self.it_count = 0
        self.int_err = 0
        self.der_err = 0
        self.int_anti_windup = 10

        self.rate = 100.0  #Hz
        self.sample_time = 1.0/self.rate 
        self.USE_FIXED_RATE = True # If true -> dT = 0.01, Else -> function call time difference 
        self.current_time = time.time()
        self.last_time = self.current_time
        self.last_error = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # error in cartesian space

        # Shared variables
        self.mutex = threading.Lock()
        self.time_limit = rospy.get_param("~time_limit", TIME_LIMIT)
        self.damping = rospy.get_param("~damping", DAMPING)
        self.joint_vel_limit = rospy.get_param("~joint_vel_limit", JOINT_VEL_LIMIT)
        self.q = np.zeros(7)        # Joint angles
        self.qdot = np.zeros(7)     # Joint velocities
        self.T_goal = np.array(self.kin.forward(self.q))    # Ref se3
        self.cur_frame = pm.fromMatrix(self.T_goal) # -> PyKDL frame
        # robot @ its original pose : position (0.3161, 0.1597, 1.151) , orientation (0.337, 0.621, 0.338, 0.621)
        self.original_goal = self.T_goal.copy() # robot @ its home pose : 
        print ('Verifying initial pose...')
        print (self.T_goal)
        self.isCalcOK = False
        self.isPathPlanned = False
        self.traj_err_bound = float(1e-2) # in meter
        self.plan_time = 2.5 # in sec. can this be random variable?
        self.rand_plan_min = 5.0
        self.rand_plan_max = 9.0
        # Subscriber
        self.ee_position = [0.0, 0.0, 0.0]
        self.ee_orientation = [0.0, 0.0, 0.0, 0.0]
        self.ee_lin_twist = [0.0, 0.0, 0.0]
        self.ee_ang_twist = [0.0, 0.0, 0.0]

        rospy.Subscriber('/demo/target/', Pose, self.ref_poseCB)
        

        self.gripper.calibrate()
        # path planning
        self.num_wp = int(0)
        self.cur_wp_idx = int(0) # [0:num_wp - 1]
        self.traj_list = [None for _ in range(self.num_wp)]
        self.traj_elapse = 0.0 # in ms

        self.r = rospy.Rate(self.rate)
        # robot chain for the ik solver of PyKDL
        # This constructor parses the URDF loaded in rosparm urdf_param into the
        # needed KDL structures. 
        self._base = self.robot.get_root()
        self._tip = "right_gripper_tip"
        self.sawyer_tree = kdl_tree_from_urdf_model(self.robot) # sawyer's kinematic tree
        self.sawyer_chain = self.sawyer_tree.getChain(self._base, self._tip)

        # KDL solvers
        self.ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self.sawyer_chain)
        self._num_jnts = 7
        self._joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

        rospy.loginfo('Check the URDF of sawyer')
        self.print_robot_description()
        rospy.loginfo('Check the sanity of kinematic chain')
        self.print_kdl_chain()
                
        self.init_frame = PyKDL.Frame() # frame of the start pose of the trajectory 
        self.integ_frame = PyKDL.Frame() # frame of the start pose of the trajectory 

        # HTC Vive related members
        self._fr_t_prev_vr = PyKDL.Frame()
        self._fr_t_cur_vr = PyKDL.Frame()
        self.pose1 = Pose()
        self.pose2 = Pose()
        self.init_Mat_vTs = np.eye(4, dtype=float) # initial homogen tf matrix; sawyer's ee seen from vr tracker
        rospy.Subscriber("test", itzy, callback=self.msgggg)
                
        self.btn_state = 0
        self.btn_state_2 = 0
        self.btn_state_3 = 0
        self._gripper = intera_interface.Gripper()
        rospy.Subscriber('vive_right', Joy, callback=self.grip_open_close)
        # control loop
        rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState , self.endpoint_poseCB)

        self._servo_home()

        self.DEBUG = False
        while not rospy.is_shutdown():
            if self.btn_state_2:
                self.calibrate_htc_to_sawyer()
            if self.btn_state_3:
                if rospy.has_param('vel_calc'):
                    self._servo_home()
            if rospy.has_param('vel_calc'):
                self.calc_joint_vel_3()
            self.r.sleep()


    def ref_poseCB(self, goal_pose): # Takes target pose, returns ref se3
        rospy.logdebug("ref_pose_cb called in velocity_control.py")
        # p = np.array([some_pose.position.x, some_pose.position.y, some_pose.position.z])
        p = np.array([goal_pose.position.x, goal_pose.position.y, goal_pose.position.z])
        quat = np.array([goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w])
        goal_tmp = tr.compose_matrix(angles=tr.euler_from_quaternion(quat, 'sxyz'), translate=p) # frame is spatial 'sxyz', return Euler angle from quaternion for specified axis sequence.
        with self.mutex:
            self.original_goal = goal_tmp


    def _get_ee_position(self):
        return self.ee_position


    def _get_goal_matrix(self):
        # print (self.cur_wp_idx)
        return self.traj_list[self.cur_wp_idx]

    def _servo_home(self):
        if rospy.has_param('vel_calc'):
            rospy.delete_param('vel_calc')
        self.move_to_start(starting_joint_angles)


    def move_to_start(self, start_angles=None):
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()


    def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            self.limb.set_joint_position_speed(0.3)
            self.limb.move_to_joint_positions(joint_angles,timeout=timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")



    def path_planning(self):
        """ Generate desriable waypoints for achieving target pose.
            - Example
            dt = 0.01
            #Create a trajectory to follow
            thetaend = [pi / 2, pi, 1.5 * pi]
            Tf = 1
            N = Tf / dt
            method = 5
            dt = Tf / (N - 1.0)
        """
        dt = self.sample_time
        current_pose = self.limb.endpoint_pose()

        X_current = self._get_tf_matrix(current_pose)
        X_goal = self.original_goal
        # X_goal[2][3] += 0.05
        rospy.logwarn('=============== Current pose ===============')
        print (X_current)
        rospy.logwarn('=============== Goal goal ===============')
        print (X_goal)
        Tf = self.plan_time
        # Tf = np.random.uniform(self.rand_plan_min, self.rand_plan_max)
        N = int(Tf / dt) # ex: plantime = 7, dt =f 0.01 -> N = 700
        self.num_wp = N
        self.traj_list = r.CartesianTrajectory(X_current, X_goal, Tf=Tf, N=N, method=CUBIC)
        self.set_init_frame() # save the robot's pose frame at start of the trajectory.
        self.isPathPlanned = True
        # self._get_target_pose()


    def _check_traj(self, dT):
        """Check if the end-effector has reached the desired position of target waypoint.
        """
        _ee_position = self._get_ee_position()
        _targ_wp_position = tr.translation_from_matrix(self.traj_list[self.cur_wp_idx])
        # if np.linalg.norm( np.array(_ee_position) -_targ_wp_position) <= self.traj_err_bound:
        if self.cur_wp_idx < self.num_wp - 1: # indicate next waypoint
            self.cur_wp_idx += 1
            self.traj_elapse += dT
        elif self.cur_wp_idx == self.num_wp - 1: # robot has reached the last waypoint
            # rospy.logwarn('Reached target object')
            self.cur_wp_idx = 0
            rospy.delete_param('vel_calc')
            self.isPathPlanned = False
            self.traj_elapse = 0


    def _get_target_pose(self):
        """ Get the episodic target pose of the object
        """
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            object_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            # object_state = object_state_srv("block", "world")
            object_state = object_state_srv("block", "base")
            print ([object_state.pose.position.x, object_state.pose.position.y, object_state.pose.position.z])
        except rospy.ServiceException as e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e)) 


    def _get_tf_matrix(self, pose):
        """Return the homogeneous matrix of given Pose
        """
        if isinstance(pose, dict):
            _quat = np.array([pose['orientation'].x, pose['orientation'].y, pose['orientation'].z, pose['orientation'].w])
            _trans = np.array([pose['position'].x, pose['position'].y, pose['position'].z])
            return tr.compose_matrix(angles=tr.euler_from_quaternion(_quat,'sxyz'), translate=_trans)
        else:
            _quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            _trans = np.array([pose.position.x, pose.position.y, pose.position.z])
            return tr.compose_matrix(angles=tr.euler_from_quaternion(_quat,'sxyz'), translate=_trans)


    def get_q_now(self):
        """ Return current joint position measurements.
        """         
        qtemp = np.zeros(7)
        i = 0
        while i<7:
            qtemp[i] = self.limb.joint_angle(self.names[i])
            i += 1
        with self.mutex:
            self.q = qtemp              # Angles in radians


    def stop_oscillating(self):
        """Nullify the joint command oscillations
        """
        i = 0
        v_norm = 0
        qvel = self.qdot

        while i<7:
            v_norm += pow(qvel[i],2)
            i += 1
        v_norm = sqrt(v_norm)

        if v_norm < 0.1:
            self.qdot = np.zeros(7)
        return


    def calc_joint_vel(self):
        """ Deprecated version of joint vel command calculation (Only feed-forward)
        """
        rospy.logdebug("Calculating joint velocities...")

        # Body stuff
        Tbs = self.M0 # 
        Blist = self.Blist # 

        # Current joint angles
        self.get_q_now()
        with self.mutex:
            q_now = self.q #1x7 mx

        # Desired config: base to desired - Tbd
        with self.mutex:
            T_sd = self.T_goal # 
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        # Find transform from current pose to desired pose, error
        # refer to CH04 >> the product of exponential formula for open-chain manipulator
        # sequence:
        #   1. FKinBody (M, Blist, thetalist)
        #       M: the home config. of the e.e.
        #       Blist: the joint screw axes in the ee frame, when the manipulator is @ the home pose
        #       thetalist : a list of current joints list
        #       We get the new transformation matrix T for newly given joint angles
        #   
        #           1). np.array(Blist)[:,i] >> retrieve one axis' joint screw 
        #           2). ex)  [s10, -c10, 0., -1.0155*c10, -1.0155*s10, -0.1603] -> S(theta)
        #           3). _out = VecTose3(_in)
        #                 # Takes a 6-vector (representing a spatial velocity).
        #                 # Returns the corresponding 4x4 se(3) matrix.
        #           4). _out = MatrixExp6(_in)
        #                    # Takes a se(3) representation of exponential coordinate 
        #                    # Returns a T matrix SE(3) that is achieved by traveling along/about the
        #                    # screw axis S for a distance theta from an initial configuration T = I(dentitiy)
        #           5). np.dot (M (from base to robot's e.e. @home pose , and multiplying exp coord(6), we get new FK pose
        #
        #   2. TransInv >> we get the inverse of homogen TF matrix
        #   3. error = np.dot (T_new, T_sd) >> TF matrix from cur pose to desired pose
        #   4. Vb >>compute the desired body twist vector from the TF matrix 
        #   5. JacobianBody:
        #           # In: Blist, and q_now >> Out : T IN SE(3) representing the end-effector frame when the joints are
        #                 at the specified coordinates
        e = np.dot(r.TransInv(r.FKinBody(Tbs, Blist, q_now)), T_sd) # Modern robotics pp 230 22Nff
        # Desired TWIST: MatrixLog6 SE(3) -> se(3) exp coord
        Vb = r.se3ToVec(r.MatrixLog6(e))  # shape : (6,)
        # Construct BODY JACOBIAN for current config
        Jb = r.JacobianBody(Blist, q_now) #6x7 mx # shape (6,7)
        # WE NEED POSITION FEEDBACK CONTROLLER
        # Desired ang vel - Eq 5 from Chiaverini & Siciliano, 1994
        # Managing singularities: naive least-squares damping
        n = Jb.shape[-1] #Size of last row, n = 7 joints
        # OR WE CAN USE NUMPY' PINV METHOD 

        invterm = np.linalg.inv(np.dot(Jb.T, Jb) + pow(self.damping, 2)*np.eye(n)) # 
         
        
        qdot_new = np.dot(np.dot(invterm,Jb.T),Vb) # It seems little bit akward...? >>Eq 6.7 on pp 233 of MR book

        self.qdot = qdot_new #1x7

        # Constructing dictionary
        qdot_output = dict(zip(self.names, self.qdot))

        # Setting Sawyer right arm joint velocities
        self.limb.set_joint_velocities(qdot_output)
        # print qdot_output
        return


    def print_robot_description(self):
        nf_joints = 0
        for j in self.robot.joints:
            if j.type != 'fixed':
                nf_joints += 1
        print "URDF non-fixed joints: %d;" % nf_joints
        print "URDF total joints: %d" % len(self.robot.joints)
        print "URDF links: %d" % len(self.robot.links)
        print "KDL joints: %d" % self.sawyer_tree.getNrOfJoints()
        print "KDL segments: %d" % self.sawyer_tree.getNrOfSegments()


    def print_kdl_chain(self):
        for idx in xrange(self.sawyer_chain.getNrOfSegments()):
            print '* ' + self.sawyer_chain.getSegment(idx).getName()


    def scale_joint_vel(self, q_dot):
        """ Scale the joint velocity no to exceed the limit.
        """
        minus_v = abs(np.amin(q_dot))
        plus_v = abs(np.amax(q_dot))
        if minus_v > plus_v:
            scale = minus_v
        else:
            scale = plus_v
        if scale > self.joint_vel_limit:
            return 1.0*(q_dot/scale)*self.joint_vel_limit
        else:
            return q_dot


    def get_dt(self):
        """ Returns the delta_T for recursive function calls
        """
        if self.USE_FIXED_RATE:
            return self.sample_time
        else:
            self.current_time = time.time()
            return (self.current_time - self.last_time) / 1000.0


    def joints_to_kdl(self, type, values=None):
        """Dtype translation list of joint values to PyKDL Vector
        """
        kdl_array = PyKDL.JntArray(self._num_jnts)
        if values is None:
            if type == 'positions':
                cur_type_values = self.limb.joint_angles()
            elif type == 'velocities':
                cur_type_values = self.limb.joint_velocities()
            elif type == 'torques':
                cur_type_values = self.limb.joint_efforts()
        else:
            cur_type_values = values
        
        for idx, name in enumerate(self._joint_names):
            kdl_array[idx] = cur_type_values[name]
        if type == 'velocities':
            kdl_array = PyKDL.JntArray(kdl_array)
        return kdl_array


    def kdl_to_mat(self, data):
        """Dtype translation from KDL frame to matrix in generic python list
        """
        mat =  np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i,j] = data[i,j]
        return mat


    def forward_velocity_kinematics(self,joint_velocities=None):
        """Solve forward vel-kine : joint-vel -> ee-vels
        """
        end_frame = PyKDL.FrameVel()
        self._fk_v_kdl.JntToCart(self.joints_to_kdl('velocities', joint_velocities),
                                 end_frame)
        return end_frame.GetTwist()


    def kdl_inv_vel_kine(self, cur_joint_pos, ee_twist=None):
        """ inverse velocity kinematics using the solver in PyKDL
            instance : self.ik_v_kdl
            refer ik_p_kdl-> ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            PyKDL -> ee frame velocity? 
                    KDL::FrameVel v_out;
                    tf::poseMsgToKDL(cart_pose, pose_kdl);
                    //                                cur_jnt_position, ee_vel
                    CartToJntVel(JointVelList[i].q, end_effector_vel.GetTwist(), NullSpaceBias[i], result_vel)
                    kin.ik_solver->CartTo
        """
        frame_vel = PyKDL.FrameVel()
        _joint_vels = self.joints_to_kdl(type='velocities', values=None)
        # args :   CartToJnt-> (cur_joint_pos, targ_ee_vel, result_joint_vel)  
        if self.ik_v_kdl.CartToJnt(cur_joint_pos, ee_twist, _joint_vels) >= 0:
            return list(_joint_vels)
        else:
            return None


    def get_feed_forward(self):
        """ Return feedforward end-effector twist.
        """
        return self.ee_ang_twist + self.ee_lin_twist


    def get_current_twist(self):
        """ Return current end-effector twist.
        """
        return [self.ee_ang_twist, self.ee_lin_twist]


    def get_current_frame(self):
        """ Create a PyKDL from current robot pose message.
        """
        return self.cur_frame


    def grip_open_close(self, j):
        prev_state = self.btn_state
        self.btn_state = j.buttons[0] # grip
        self.btn_state_2 = j.buttons[1] # calibrate
        self.btn_state_3 = j.buttons[3] # delete ros param
        diff = self.btn_state - prev_state 
        if diff == 1: # clicked
            self.gripper_close()
        elif diff == -1:
            self.gripper_open()


    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    
    def msgggg(self, data):
        self.pose1 = data.ymg                 # vr_t-1_Pose
        self.pose2 = data.ymg_2               # vr_t_Pose
        self._fr_t_prev_vr = pm.fromMsg(self.pose1) # vr_t-1_PyKDL.Frame
        self._fr_t_cur_vr = pm.fromMsg(self.pose2) # vr_t_PyKDL.Frame

    
    def get_des_frame_from_vr(self):
        """Return the desired frame for the Sawyer to follow in a single sample step.
            dtype : ndarray
        """
        return pm.toMatrix(self._fr_t_cur_vr)

   
    def endpoint_poseCB(self, state):
        """ receives end-effectors' pose and twists for feedback-purpose
        """
        ee_pose = state.pose
        ee_pose.orientation = overhead_orientation
        ee_twist = state.twist

        self.ee_position = [ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]
        self.ee_orientation = [ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w]

        # self.ee_orientation = [0.00142460053167, 0.999994209902, -0.00177030764765, 0.00253311793936]
        self.ee_lin_twist = [ee_twist.linear.x, ee_twist.linear.y, ee_twist.linear.z]
        self.ee_ang_twist = [ee_twist.angular.x, ee_twist.angular.y, ee_twist.angular.z]
        
        self.cur_frame = pm.fromMsg(ee_pose)  # sawyer_t_PyKDL.Frame


    def calibrate_htc_to_sawyer(self):
        """ execute the following linear-algebraic operation.
        for : 
            {b} - base frame (same for both the HTC and Sawyer)
            {v} - tracker of the HTC vive frame (initilly has offset transform of vTs (Sawyer'pose seen from tracker frame))
            {s} - ee frame of the sawyer (gripper_tip) 
        """
        _cur_fr = self.get_current_frame()
        b_T_s0 = pm.toMatrix(_cur_fr)
        b_T_v0 = self.get_des_frame_from_vr()
        b_T_v0_inv = inv(b_T_v0)
        self.init_Mat_vTs = np.dot(inv(b_T_v0), b_T_s0) # numpy 4x4 array
        self.set_init_frame() # save the robot's pose frame at start of the trajectory.
        if self.DEBUG:
            rospy.logwarn("________________________________DEBUG REPORTS___________________________________")
            print ('====== base to Sawyer ======')
            print (b_T_s0)

            print ('====== base to Vive ======')
            print (b_T_v0)
            print ('====== Vive inversed ======')
            print (b_T_v0_inv)
            print ('====== Transfrom matrix ======')
            print (self.init_Mat_vTs)
            print ('====== calibrated base to Sawyer ======')
            calib = np.dot(b_T_v0, self.init_Mat_vTs)
            print (calib)
            self.get_err_twist()
            rospy.logwarn("________________________________________________________________________________")
        print ('===================================================')
        rospy.loginfo(" It's OK to deploy sawyer. vel cal starts in 3 secs")
        print (self.init_Mat_vTs)
        rospy.sleep(3.0)
        rospy.set_param('vel_calc', True)
       

    def get_err_twist(self, goal_frame = None, dT=0.01):
        """ Get the desirable twist for given error.
        """
        _des_fr_vr = self.get_des_frame_from_vr()
        _des_fr_sw = np.dot(_des_fr_vr, self.init_Mat_vTs)
        cur_fr_sw = self.get_current_frame()
        des_fr_sw = pm.fromMatrix(_des_fr_sw)
        if self.DEBUG:
            rospy.logwarn("________________________________DEBUG REPORTS___________________________________")
            print ('====== Desried frame for vr ======')
            print (_des_fr_vr)     
            print ('====== Desried frame for sawyer ======')
            print (_des_fr_sw)
            print ('====== Current frame for sawyer ======')
            print (cur_fr_sw)
            rospy.logwarn("________________________________________________________________________________")

        return PyKDL.diff(cur_fr_sw, des_fr_sw, dT) 


    def get_des_twist(self, dT=0.01):
        """ Get desired twist @ time t, by computing differential of traj_frame @ t=T-1 and t=T
        """
        return PyKDL.diff(self._fr_t_prev_vr, self._fr_t_cur_vr, dT)            


    def integ_error(self, twist_err, dT=0.01):
        """Apply timestep-wise error integration.
        """
        self.integ_frame = PyKDL.addDelta(self.integ_frame, twist_err, dT)
        return PyKDL.diff(self.init_frame, self.integ_frame, self.traj_elapse)




    def calc_joint_vel_3(self):
        """ calc joint vel using PyKDL IK
        """
        dT = self.get_dt()
        rospy.logwarn('========== Debug delta T ==========')
        print (dT)
        rospy.logwarn('===================================')
        with self.mutex:
            q_now = self.joints_to_kdl(type='positions', values=None) # Dtype translation list of joint values to PyKDL Vector
        err_twist = self.get_err_twist() # err term twist(Xd - X)
        
        des_twist = self.get_des_twist() # feed-forward twist value Vd_dot
        
        rospy.logwarn('=============== Twist error ===============')
        print (err_twist)
        rospy.logwarn('=============== Twist desired ===============')
        print (des_twist)
        integ_twist = self.integ_error(err_twist)
        rospy.logwarn('=============== Twist integtaed ===============')
        print (integ_twist)
        err_twist, integ_twist = self.apply_gain(err_twist, integ_twist)
        print('des_twist',   des_twist)
        print('err_twist',   err_twist)
        print('integ_twist', integ_twist)
        total_twist = des_twist + err_twist + integ_twist # FF + Pg*Err + Ig*Integ(Err)
        
        self.q_dot = self.kdl_inv_vel_kine(cur_joint_pos=q_now, ee_twist=total_twist)
        self.q_dot = self.scale_joint_vel(self.q_dot)
        
        # publish joint command 
        qdot_output = dict(zip(self.names, self.q_dot))
        # if not self.DEBUG:
        self.limb.set_joint_velocities(qdot_output)
                
        # dT = 0.01
        self.traj_elapse += dT


    def get_frame(self, matrix):
        """ Convert the generic homogen. transformation matrix to geometrically same PyKDL frame.
            TODO: how to convert 
        """
        # _r, _t = r.TransToRp(matrix)
        # _rot = PyKDL.Rotation(_r[0][0], _r[0][1], _r[0][2],
        #                            _r[1][0], _r[1][1], _r[1][2],
        #                            _r[2][0], _r[2][1], _r[2][2])
        # _tran = PyKDL.Vector(_t[0], _t[1], _t[2])
        return pm.fromMatrix(np.array(matrix))


    def calc_joint_vel_2(self):
        """ Joint velocity command computation based on PI feedback loop.
        """
        Tbs = self.M0 # 
        Blist = self.Blist # 
        self.get_q_now()
        with self.mutex:
            q_now = self.q #1x7 mx
        with self.mutex:
            T_sd = self._get_goal_matrix() # 
        dT = self.get_dt()
        e = np.dot(r.TransInv(r.FKinBody(Tbs, Blist, q_now)), T_sd) # Modern robotics pp 230 22Nff
        # get Error vector.
        Xe = r.se3ToVec(r.MatrixLog6(e)) # shape -> [1, 2, 3, 4, 5, 6]
        dXe = Xe - self.last_error
        # numerical integration of the error
        if np.linalg.norm(self.int_err) < self.int_anti_windup:
            self.int_err += Xe # * dT
        # numerical differentiation of the error
        if dT > 0:
            self.der_err = dXe / dT
        Vb = np.dot(self.Kp, Xe) + np.dot(self.Ki, self.int_err) #+ np.dot(self.Kd, self.der_err) # Kp*Xe + Ki*intg(Xe) + Kd*(dXe/dt)
        Vb += self.get_feed_forward()
        # self.V_b = np.dot(self.Kp, self.X_e) + np.dot(self.Ki, self.int_err)
        self.J_b = r.JacobianBody(Blist, self.q)
        n = self.J_b.shape[-1] #Size of last row, n = 7 joints
        invterm = np.linalg.inv(np.dot(self.J_b.T, self.J_b) + pow(self.damping, 2)*np.eye(n)) # 
        self.q_dot = np.dot(np.dot(invterm, self.J_b.T),Vb) # It seems little bit akward...? >>Eq 6.7 on pp 233 of MR book
        self.q_dot = self.scale_joint_vel(self.q_dot)
        qdot_output = dict(zip(self.names, self.q_dot))
        self.limb.set_joint_velocities(qdot_output)
        # self.stop_oscillating()
        self.last_error = Xe
        self.last_time = self.current_time
        self._check_traj()


    def set_init_frame(self):
        """ Set the frame of the start pose of the trajectory.
        """
        self.init_frame = self.get_current_frame()

    def apply_gain(self, prop_err, integ_err):
        Kp = 0.0
        Ki = 0.0
        return Kp * prop_err, Ki * integ_err




def main():
    rospy.init_node('velocity_control')

    try:
        vc = VelocityControl()
        p_t = time.time()
        p_t = p_t + 3
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

if __name__ == '__main__':
    main()
    
