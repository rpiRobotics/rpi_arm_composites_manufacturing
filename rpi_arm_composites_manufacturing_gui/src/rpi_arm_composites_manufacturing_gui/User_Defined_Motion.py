import numpy as np
import copy
import rospy
import actionlib
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
import general_robotics_toolbox as rox
import general_robotics_toolbox.urdf as urdf
import general_robotics_toolbox.ros_msg as rox_msg
from pyspin_wrapper.srv import CameraTrigger

from cv_bridge import CvBridge, CvBridgeError
from industrial_payload_manager import PayloadTransformListener

import tf
import time
import sys
from scipy.linalg import expm
import cv2
import cv2.aruco as aruco
import scipy.misc
from scipy.io import loadmat

import rpi_abb_irc5.ros.rapid_commander as rapid_node_pkg
import safe_kinematic_controller.ros.commander as controller_commander_pkg
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal
from safe_kinematic_controller.msg import ControllerState as controllerstate

from moveit_msgs.msg import RobotTrajectory

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from QuadProg_YC import QP_abbirb6640


ft_threshold=[250,250,250,250,250,250]
ft_threshold_place=[500,500,500,500,500,500]
FTdata = None


def trapgen(xo,xf,vo,vf,vmax,amax,dmax,t):

#Generate trapezoidal velocity motion profile.
#
#inputs:
#xo: initial position
#xf: final position
#vo: initial velocity
#vf: final velocity
#vmax: velocity limit
#amax: upper acceleration limit (magnitude)
#dmax: lower acceleration limit (magnitude)
#t: sample time
#
#outputs:
#x: position at sample time t
#v: velocity at sample time t
#a: acceleration at sample time t
#ta: first switching time
#tb: second switching time
#tf: final time

  # vo and vf must be less than vmax
    if (abs(vo)>=abs(vmax))|(abs(vf)>=abs(vmax)):
        print('vo or vf too large')
    
    vmax=np.sign(xf-xo)*vmax
    
    if xf>xo:
        am1=abs(amax)
        am2=-abs(dmax)
    else:
        am1=-abs(dmax)
        am2=abs(amax)
        
    ta=abs((vmax-vo)/am1)
    tf_tb=(vf-vo-am1*ta)/am2
    print (am1*ta+vo)
    tf=(xf-xo+.5*am1*ta**2-.5*am2*tf_tb**2)/(am1*ta+vo)
    tb=tf-tf_tb
    
    if ((tf<2*ta)|(tb<ta)):
        #tapoly=[1, 2*vo/am1 ((vf**2-vo**2)/2/am2+xo-xf)*2/(1-am1/am2)/am1]
        ta = -vo/am1 + np.sqrt((vo/am1)**2-	(2/(1-am1/am2)/am1)*((vf-vo)**2/2/am2+xo-xf))
        tf=(vf-vo-am1*ta)/am2+ta
        tb=ta
    
    if t<ta:
        a=am1
        v=am1*t+vo
        x=.5*am1*t**2+vo*t+xo
    elif t<tb:
        a=0
        v=am1*ta+vo;
        x=am1*(t*ta-.5*ta**2)+vo*t+xo;
    elif t<=tf:
        a=am2
        v=am2*(t-tb)+am1*ta+vo
        x=am1*(-.5*ta**2+t*ta)+am2*(.5*t**2+.5*tb**2-tb*t)+vo*t+xo
    else:
        a=0
        v=0
        x=xf


    return x,v,a,ta,tb,tf


def trapezoid_gen(target,current_joint_angles,acc,dx):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    goal.trajectory.header.frame_id='/world'

    dist = np.array(target-current_joint_angles)
    duration = np.max(np.sqrt(np.abs(9.0*dist/2.0/acc)))
    
    vmax = 1.5/duration
    amax  = 3*vmax/duration
    dmax  = amax
    [x0,v0,a0,ta,tb,tf] = trapgen(0,1,0,0,vmax,amax,dmax,0)
    [xa,va,aa,ta,tb,tf] = trapgen(0,1,0,0,vmax,amax,dmax,ta)
    [xb,vb,ab,ta,tb,tf] = trapgen(0,1,0,0,vmax,amax,dmax,tb)
	
    #print 'input:',dist,acc,9.0*dist/2.0/acc
	
    duration = np.max(np.sqrt(np.abs(9.0*dist/2.0/acc)))
    vmax = 1.5*dist/duration
    acc = 3*vmax/duration
    print 'duration',duration,dist
	
    p1=JointTrajectoryPoint()
    p1.positions = current_joint_angles
    p1.velocities = np.zeros((6,))
    p1.accelerations = aa*dist
    p1.time_from_start = rospy.Duration(0)
    
    p2=JointTrajectoryPoint()
    p2.positions = np.array(p1.positions) + dist*xa
    p2.velocities = va*dist
    p2.accelerations = np.zeros((6,))
    p2.time_from_start = rospy.Duration(ta)
    
    p3=JointTrajectoryPoint()
    p3.positions = np.array(p1.positions) + dist*xb
    p3.velocities = vb*dist
    p3.accelerations = -ab*dist
    p3.time_from_start = rospy.Duration(tb)

    p4=JointTrajectoryPoint()
    p4.positions = target
    p4.velocities = np.zeros((6,))
    p4.accelerations = np.zeros((6,))
    p4.time_from_start = rospy.Duration(tf)
    
    goal.trajectory.points.append(p1)
    goal.trajectory.points.append(p2)
    goal.trajectory.points.append(p3)
    goal.trajectory.points.append(p4)
    
    return goal


class CameraParams:
    def __init__(self,M00,M02,M11,M12,M22,C00,C01,C02,C03,C04):
        self.camMatrix, self.distCoeff = self.CameraParams(M00,M02,M11,M12,M22,C00,C01,C02,C03,C04)


    def CameraParams(self, M00,M02,M11,M12,M22,C00,C01,C02,C03,C04):
        camMatrix = np.zeros((3, 3),dtype=np.float64)
        camMatrix[0][0] = M00
        camMatrix[0][2] = M02
        camMatrix[1][1] = M11
        camMatrix[1][2] = M12
        camMatrix[2][2] = M22

        distCoeff = np.zeros((1, 5), dtype=np.float64)
        distCoeff[0][0] = C00
        distCoeff[0][1] = C01
        distCoeff[0][2] = C02
        distCoeff[0][3] = C03
        distCoeff[0][4] = C04

#        params = {'camMatrix': camMatrix, 'distCoeff': distCoeff}
        return camMatrix,distCoeff

class ObjectRecognitionCommander(object):
    def __init__(self):
        self.client=actionlib.SimpleActionClient("recognize_objects_overhead", ObjectRecognitionAction)
        self.listener=PayloadTransformListener()
        self.ros_image=None
        self.ros_image_stamp=rospy.Time(0)
        self.bridge = CvBridge()
    def get_object_pose(self, key):
        self.client.wait_for_server()
        
        goal=ObjectRecognitionGoal(False, [-1e10,1e10,-1e10,1e10])
        self.client.send_goal(goal)
        self.client.wait_for_result()
        ret=self.client.get_result()
        
        for r in ret.recognized_objects.objects:
            if r.type.key == key:
                return rox_msg.msg2pose(r.pose.pose.pose)
            
        raise Exception("Requested object not found")
    
    def get_object_gripper_target_pose(self, key):
        
        object_pose=self.get_object_pose(key)
                
        (trans1,rot1) = self.listener.lookupTransform(key, key + "_gripper_target", rospy.Time(0))
        tag_rel_pose=rox.Pose(rox.q2R([rot1[3], rot1[0], rot1[1], rot1[2]]), trans1)
        return object_pose * tag_rel_pose

    def ros_raw_gripper_2_image_cb(self, ros_image_data):
        self.ros_image = self.bridge.imgmsg_to_cv2(ros_image_data, desired_encoding="passthrough")
        self.ros_image_stamp= ros_image_data.header.stamp

def callback(data):
    global FTdata
    FTdata = np.array([data.ft_wrench.torque.x,data.ft_wrench.torque.y,data.ft_wrench.torque.z,\
    data.ft_wrench.force.x,data.ft_wrench.force.y,data.ft_wrench.force.z])

def main():
            
    #Start timer to measure execution time        
    t1 = time.time()
    
    #Subscribe to controller_state 
    rospy.Subscriber("controller_state", controllerstate, callback)
    last_ros_image_stamp = rospy.Time(0)
    
    #Force-torque    
    if not "disable-ft" in sys.argv:
        ft_threshold1=ft_threshold
    else:
        ft_threshold1=[]
    
    #Initialize ros node of this process
    rospy.init_node('user_defined_motion', anonymous=True)
    
    #print "============ Starting setup"   
    
    listener = PayloadTransformListener()
    rapid_node = rapid_node_pkg.RAPIDCommander()
    controller_commander = controller_commander_pkg.ControllerCommander()

    object_commander=ObjectRecognitionCommander()
    robot = urdf.robot_from_parameter_server()   
  
    #subscribe to Gripper camera node for image acquisition     
    ros_gripper_2_img_sub=rospy.Subscriber('/gripper_camera_2/image', Image, object_commander.ros_raw_gripper_2_image_cb)
    ros_gripper_2_trigger=rospy.ServiceProxy('/gripper_camera_2/continuous_trigger', CameraTrigger)

    #Set controller command mode
    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.4, ft_threshold_place,[])
    time.sleep(0.5)
    
    
    #Set Location above the panel where the end effector goes first (in the world/base frame) Ideal location of panel.
    current_joint_angles = controller_commander.get_current_joint_values()
    Cur_Pose = controller_commander.get_current_pose_msg()
    rot_o = [Cur_Pose.pose.orientation.w, Cur_Pose.pose.orientation.x,Cur_Pose.pose.orientation.y,Cur_Pose.pose.orientation.z]
    trans_o = np.array([Cur_Pose.pose.position.x,Cur_Pose.pose.position.y,Cur_Pose.pose.position.z])

    pos_command = []
    quat_command = []
    pos_read = []
    quat_read = []
    time_all = []


    for i in range(4):
        rot_set = rox.q2R(rot_o)
        tran_set = trans_o+np.array([0,0,i*0.25])


        pose_target2 = rox.Transform(rot_set, tran_set)
    
         
        #Execute movement to set location
        print "Executing initial path"
        controller_commander.compute_cartesian_path_and_move(pose_target2, avoid_collisions=False)
        time.sleep(1)
    
        Cur_Pose = controller_commander.get_current_pose_msg()
        rot_cur = [Cur_Pose.pose.orientation.w, Cur_Pose.pose.orientation.x,Cur_Pose.pose.orientation.y,Cur_Pose.pose.orientation.z]
        trans_cur = [Cur_Pose.pose.position.x,Cur_Pose.pose.position.y,Cur_Pose.pose.position.z]

        pos_command.append(tran_set)
        quat_command.append(rot_set)
        pos_read.append(trans_cur)
        quat_read.append(rot_cur)
        time_all.append(time.time())
    

    #Saving iteration data to file
    filename_data = "/home/rpi-cats/Desktop/YC/Data/Motion Capture/Data_"+str(t1)+".mat"
    scipy.io.savemat(filename_data, mdict={'pos_command':pos_command, 'quat_command':quat_command, 'pos_read':pos_read, 'quat_read':quat_read, 'time_all':time_all})
	

    t2 = time.time()
    print 'Execution Finished.'
    print "Execution time: " + str(t2-t1) + " seconds"

if __name__ == '__main__':
    main()
