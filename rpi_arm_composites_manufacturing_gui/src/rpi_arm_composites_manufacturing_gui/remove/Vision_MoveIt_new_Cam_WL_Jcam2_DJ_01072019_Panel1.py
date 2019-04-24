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
from rpi_arm_composites_manufacturing_process.msg import ProcessStepAction, ProcessStepGoal, ProcessState
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
#from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory
from QuadProg_YC import QP_abbirb6640

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ft_threshold=[500,500,500,500,500,500]
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
    [x0,v0,a0,ta,tb,t_f] = trapgen(0,1,0,0,vmax,amax,dmax,0)
    [xa,va,aa,ta,tb,t_f] = trapgen(0,1,0,0,vmax,amax,dmax,ta)
    [xb,vb,ab,ta,tb,t_f] = trapgen(0,1,0,0,vmax,amax,dmax,tb)
	
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
    p4.time_from_start = rospy.Duration(t_f)
    
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
    last_ros_image_stamp=rospy.Time(0)
    
    #Force-torque    
    if not "disable-ft" in sys.argv:
        ft_threshold1=ft_threshold
    else:
        ft_threshold1=[]
    
    #Initialize ros node of this process
    rospy.init_node('Placement_DJ_1', anonymous=True)
    process_client=actionlib.SimpleActionClient('process_step', ProcessStepAction)
    process_client.wait_for_server()
    listener = PayloadTransformListener()
    rapid_node = rapid_node_pkg.RAPIDCommander()
    controller_commander=controller_commander_pkg.ControllerCommander()

    object_commander=ObjectRecognitionCommander()
    robot = urdf.robot_from_parameter_server()   
  
    #subscribe to Gripper camera node for image acquisition     
    ros_gripper_2_img_sub=rospy.Subscriber('/gripper_camera_2/image', Image, object_commander.ros_raw_gripper_2_image_cb)
    ros_gripper_2_trigger=rospy.ServiceProxy('/gripper_camera_2/continuous_trigger', CameraTrigger)

    #Set controller command mode
    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.4, [],[])
    time.sleep(0.5)
    
    ''' 
    #open loop set the initial pose
    #Set Location above the panel where the end effector goes first (in the world/base frame) Ideal location of panel.
    #Cur_Pose = controller_commander.get_current_pose_msg()
    #print Cur_Pose
    #raw_input("confirm release vacuum")

    tran = np.array([2.17209718963,-1.13702651252,0.224273328841])
    rot = rox.q2R([0.0145063655084, -0.0282932350764, 0.999322921073, -0.0185137145776])
    #tran = np.array([2.26476414097,-1.22419669418,0.411353037002])
    #rot = np.array([[-0.9959393, -0.0305329,  0.0846911], [-0.0310315,  0.9995079, -0.0045767], [-0.0845097, -0.0071862, -0.9963967]])
    pose_target2 = rox.Transform(rot, tran)
    pose_target3 = copy.deepcopy(pose_target2)
    
    ##Execute movement to set location
    print "Executing initial path"
    controller_commander.compute_cartesian_path_and_move(pose_target2, avoid_collisions=False)
    '''

    #Initilialize aruco boards and parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters =  cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.adaptiveThreshWinSizeMax=30
    parameters.adaptiveThreshWinSizeStep=7
	
	 # 1st Panel tag info 
    board_ground = cv2.aruco.GridBoard_create(4, 4, .04, .0075, aruco_dict, 32)
    board_panel = cv2.aruco.GridBoard_create(8, 3, .025, .0075, aruco_dict, 80)
    #Load object points ground tag in panel tag coordinate system from mat file
    loaded_object_points_ground_in_panel_system_stage_1 = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_Above_Nest.mat')['object_points_ground_in_panel_tag_system']
    loaded_object_points_ground_in_panel_system_stage_2 = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['object_points_ground_in_panel_tag_system']    
    loaded_object_points_ground_in_panel_system=loaded_object_points_ground_in_panel_system_stage_1


#    #Initialize camera intrinsic parameters #18285636_10_05_2018_5_params
    #18285636_10_05_2018_5_params_111122018
    CamParam = CameraParams(2283.391289766133, 1192.255485086403, 2279.484382094687, 1021.399092147012, 1.0, -0.022101211408596, -0.095163053709332, -0.003986991791212,  -0.003479613658352, 0.179926705467534)

    #Subscribe tp controller state. Again?
    rospy.Subscriber("controller_state", controllerstate, callback)

    UV = np.zeros([150,8])
    P = np.zeros([150,3])

    #focal length in pixel units, this number is averaged values from f_hat for x and y
    f_hat_u = 2283.391289766133#2342.561249990927#2282.523358266698#2281.339593273153 #2446.88
    f_hat_v = 2279.484382094687#2338.448312671424#2280.155828279608
    #functions like a gain, used with velocity to track position
    dt = 0.1
    du = 100.0
    dv = 100.0
    dutmp=100.0
    dvtmp=100.0
    #TimeGain = [0.1,0.1, 0.1]
    du_array=[]
    dv_array=[]
    dx_array=[]
    iteration=0
    stage=1
    #step_ts = 0.004
    Kc = 0.0002
    #time_save = []
    #FTdata_save = []
    Tran_z = np.array([[0,0,-1],[0,-1,0],[1,0,0]])    
    Vec_wrench = 100*np.array([0.019296738361905,0.056232033265447,0.088644197659430,    
    0.620524934626544,-0.517896661195076,0.279323567303444,-0.059640563813256,   
    0.631460085138371,-0.151143175570223,-6.018321330845553]).transpose()
 
    FTdata_0 = FTdata
    T = listener.lookupTransform("base", "link_6", rospy.Time(0))
    rg = 9.8*np.matmul(np.matmul(T.R,Tran_z).transpose(),np.array([0,0,1]).transpose())
    A1 = np.hstack([rox.hat(rg).transpose(),np.zeros([3,1]),np.eye(3),np.zeros([3,3])])
    A2 = np.hstack([np.zeros([3,3]),rg.reshape([3,1]),np.zeros([3,3]),np.eye(3)])
    A = np.vstack([A1,A2])
    FTdata_0est = np.matmul(A,Vec_wrench)
    FTread_array=[]
    FTdata_array=[]
    t_now_array=[]
    step_size = []
	
    cv2.namedWindow('Image',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Image', 490,410)

    
    ### PBVS set the initial pose
    #### Final Nest Placement Error Calculation ===============================
    #Read new image
    last_ros_image_stamp = object_commander.ros_image_stamp        
    try:
        ros_gripper_2_trigger.wait_for_service(timeout=0.1)
        ros_gripper_2_trigger(False)
    except:
        pass
    wait_count=0
    while object_commander.ros_image is None or object_commander.ros_image_stamp == last_ros_image_stamp:
        if wait_count > 50:
            raise Exception("Image receive timeout")
        time.sleep(0.25)
        wait_count += 1
    result = object_commander.ros_image    
    
    #Detect tag corners in aqcuired image using aruco
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(result, aruco_dict, parameters=parameters)
    
    #Sort corners and ids according to ascending order of ids
    corners_original=copy.deepcopy(corners)
    ids_original=np.copy(ids)
    sorting_indices=np.argsort(ids_original,0)
    ids_sorted=ids_original[sorting_indices]
    ids_sorted=ids_sorted.reshape([len(ids_original),1])
    combined_list=zip(np.ndarray.tolist(ids.flatten()),corners_original)
    combined_list.sort()
    corners_sorted=[x for y,x in combined_list]
    ids=np.copy(ids_sorted)
    corners = copy.deepcopy(corners_sorted)
    
    #Remove ids and corresponsing corners not in range (Parasitic detections in random locations in the image)
    #false_ids_ind = np.where(ids>73)       
    mask = np.ones(ids.shape, dtype=bool)        
    #mask[false_ids_ind] = False
    ids = ids[mask]        
    corners = np.array(corners)        
    corners = corners[mask.flatten(),:]        
    corners = list(corners)
    
    #Define object and image points of both tags        
    objPoints_ground, imgPoints_ground	=	aruco.getBoardObjectAndImagePoints(board_ground, corners, ids)
    objPoints_panel, imgPoints_panel	=	aruco.getBoardObjectAndImagePoints(board_panel, corners, ids)
    objPoints_ground=objPoints_ground.reshape([objPoints_ground.shape[0],3])
    imgPoints_ground=imgPoints_ground.reshape([imgPoints_ground.shape[0],2])
    objPoints_panel=objPoints_panel.reshape([objPoints_panel.shape[0],3])
    imgPoints_panel=imgPoints_panel.reshape([imgPoints_panel.shape[0],2]) 

    #Save pose of marker boards after the iterations end(while in the final hovering position above nest)
    #Get pose of both ground and panel markers from detected corners        
    retVal_ground, rvec_ground, tvec_ground = cv2.solvePnP(objPoints_ground, imgPoints_ground, CamParam.camMatrix, CamParam.distCoeff)
    Rca_ground, b_ground = cv2.Rodrigues(rvec_ground)
    retVal_panel, rvec_panel, tvec_panel = cv2.solvePnP(objPoints_panel, imgPoints_panel, CamParam.camMatrix, CamParam.distCoeff)
    Rca_panel, b_panel = cv2.Rodrigues(rvec_panel)
    
    print"============== Observed Pose difference in nest position"
    observed_tvec_difference=tvec_ground-tvec_panel
    observed_rvec_difference=rvec_ground-rvec_panel
    print "tvec difference: ",observed_tvec_difference
    print "rvec differnece: ",observed_rvec_difference
    
    #Load ideal pose differnece information from file
    loaded_rvec_difference = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_Above_Nest.mat')['rvec_difference']
    loaded_tvec_difference = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_Above_Nest.mat')['tvec_difference']

    print"============== Ideal Pose difference in nest position"
    print "tvec difference: ",loaded_tvec_difference
    print "rvec differnece: ",loaded_rvec_difference

    print"============== Difference in pose difference in nest position"
    tvec_err = loaded_tvec_difference-observed_tvec_difference
    rvec_err = loaded_rvec_difference-observed_rvec_difference 
    print "tvec difference: ",tvec_err
    print "rvec differnece: ",rvec_err

    # Adjustment
    print "Adjustment ===================="
    current_joint_angles = controller_commander.get_current_joint_values()
    dx = np.array([0,0,0, -tvec_err[0], tvec_err[1]+0.03,tvec_err[2]])
    joints_vel = QP_abbirb6640(np.array(current_joint_angles).reshape(6, 1),np.array(dx))
    goal = trapezoid_gen(np.array(current_joint_angles) + joints_vel.dot(1),np.array(current_joint_angles),0.25,np.array(dx))
    client = actionlib.SimpleActionClient("joint_trajectory_action", FollowJointTrajectoryAction)
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result()
    res = client.get_result()
    if (res.error_code != 0):
        raise Exception("Trajectory execution returned error")

    print res
    #raw_input("confirm move...")
    print "End of Initial Pose ===================="

    ### End of initial pose 
    

    step_size_min = 100000
    stage = 2
    loaded_object_points_ground_in_panel_system=loaded_object_points_ground_in_panel_system_stage_2   

    while ((du>4) | (dv>4) and (iteration<55)):    #try changing du and dv to lower values(number of iterations increases)
        iteration += 1
        t_now_array.append(time.time())

        #Go to stage2 of movement(mostly downward z motion)
        if((du<4) and (dv<4) and (stage==1)):
			#Save pose of marker boards after the iterations end(while in the final hovering position above nest)
			#Get pose of both ground and panel markers from detected corners        
			retVal_ground, rvec_ground, tvec_ground = cv2.solvePnP(objPoints_ground, imgPoints_ground, CamParam.camMatrix, CamParam.distCoeff)
			Rca_ground, b_ground = cv2.Rodrigues(rvec_ground)
			retVal_panel, rvec_panel, tvec_panel = cv2.solvePnP(objPoints_panel, imgPoints_panel, CamParam.camMatrix, CamParam.distCoeff)
			Rca_panel, b_panel = cv2.Rodrigues(rvec_panel)

			print"============== Observed Pose difference in hovering position"
			observed_tvec_difference=tvec_ground-tvec_panel
			observed_rvec_difference=rvec_ground-rvec_panel
			print "tvec difference: ",observed_tvec_difference
			print "rvec differnece: ",observed_rvec_difference

			#Load ideal pose differnece information from file
			loaded_rvec_difference = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_Above_Nest.mat')['rvec_difference']
			loaded_tvec_difference = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_Above_Nest.mat')['tvec_difference']

			print"============== Ideal Pose difference in hovering position"
			print "tvec difference: ",loaded_tvec_difference
			print "rvec differnece: ",loaded_rvec_difference
			
			tvec_difference_Above_Nest=loaded_tvec_difference-observed_tvec_difference
			rvec_difference_Above_Nest=loaded_rvec_difference-observed_rvec_difference

			print"============== Difference in pose difference in hovering position"
			print "tvec difference: ",tvec_difference_Above_Nest
			print "rvec differnece: ",rvec_difference_Above_Nest 

			#Saving pose information to file
			filename_pose1 = "/home/rpi-cats/Desktop/DJ/Code/Data/Panel1_Above_Nest_Pose_"+str(t1)+".mat"
			scipy.io.savemat(filename_pose1, mdict={'tvec_ground_Above_Nest':tvec_ground, 'tvec_panel_Above_Nest':tvec_panel, 
			'Rca_ground_Above_Nest':Rca_ground, 'Rca_panel_Above_Nest': Rca_panel, 'tvec_difference_Above_Nest': tvec_ground-tvec_panel, 'rvec_difference_Above_Nest': rvec_ground-rvec_panel,
			'loaded_tvec_difference':loaded_tvec_difference, 'loaded_rvec_difference':loaded_rvec_difference,'observed_tvec_difference':observed_tvec_difference,'observed_rvec_difference':observed_rvec_difference})
			
			#raw_input("Confirm Stage 2")        
			stage=2
#			dt=0.02
			loaded_object_points_ground_in_panel_system=loaded_object_points_ground_in_panel_system_stage_2
        #print pose_target2.p[2]
        
        #Print current robot pose at the beginning of this iteration
        Cur_Pose = controller_commander.get_current_pose_msg()
        print "============ Current Robot Pose"
        print Cur_Pose
        
        #Read new image
        last_ros_image_stamp = object_commander.ros_image_stamp        
        try:
            ros_gripper_2_trigger.wait_for_service(timeout=0.1)
            ros_gripper_2_trigger(False)
        except:
            pass
        wait_count=0
        while object_commander.ros_image is None or object_commander.ros_image_stamp == last_ros_image_stamp:
            if wait_count > 50:
                raise Exception("Image receive timeout")
            time.sleep(0.25)
            wait_count += 1
        result = object_commander.ros_image
        #Save
#        filename = "Acquisition3_%d.jpg" % (time.time())
#        scipy.misc.imsave(filename, result)
        #Display
#        cv2.namedWindow('Aqcuired Image',cv2.WINDOW_NORMAL)
#        cv2.imshow('Acquired Image',result)
#        cv2.waitKey(1)
        

        #Detect tag corners in aqcuired image using aruco
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(result, aruco_dict, parameters=parameters)
        frame_with_markers_and_axis=result
        
        #Sort corners and ids according to ascending order of ids
        corners_original=copy.deepcopy(corners)
        ids_original=np.copy(ids)
        sorting_indices=np.argsort(ids_original,0)
        ids_sorted=ids_original[sorting_indices]
        ids_sorted=ids_sorted.reshape([len(ids_original),1])
        combined_list=zip(np.ndarray.tolist(ids.flatten()),corners_original)
        combined_list.sort()
        corners_sorted=[x for y,x in combined_list]
        ids=np.copy(ids_sorted)
        corners=copy.deepcopy(corners_sorted)
        
        #Remove ids and corresponsing corners not in range (Parasitic detections in random locations in the image)
        false_ids_ind = np.where(ids>150)       
        mask = np.ones(ids.shape, dtype=bool)        
        mask[false_ids_ind] = False
        ids = ids[mask]        
        corners = np.array(corners)        
        corners = corners[mask.flatten(),:]        
        corners = list(corners)
        
        #Define object and image points of both tags        
        objPoints_ground, imgPoints_ground	=	aruco.getBoardObjectAndImagePoints(board_ground, corners, ids)
        objPoints_panel, imgPoints_panel	=	aruco.getBoardObjectAndImagePoints(board_panel, corners, ids)
        objPoints_ground=objPoints_ground.reshape([objPoints_ground.shape[0],3])
        imgPoints_ground=imgPoints_ground.reshape([imgPoints_ground.shape[0],2])
        objPoints_panel=objPoints_panel.reshape([objPoints_panel.shape[0],3])
        imgPoints_panel=imgPoints_panel.reshape([imgPoints_panel.shape[0],2]) 
        
        #Get pose of both ground and panel markers from detected corners        
        retVal_ground, rvec_ground, tvec_ground = cv2.solvePnP(objPoints_ground, imgPoints_ground, CamParam.camMatrix, CamParam.distCoeff)
        Rca_ground, b_ground = cv2.Rodrigues(rvec_ground)
        retVal_panel, rvec_panel, tvec_panel = cv2.solvePnP(objPoints_panel, imgPoints_panel, CamParam.camMatrix, CamParam.distCoeff)
        Rca_panel, b_panel = cv2.Rodrigues(rvec_panel)

        frame_with_markers_and_axis	=	cv2.aruco.drawAxis(	frame_with_markers_and_axis,  CamParam.camMatrix, CamParam.distCoeff, Rca_ground, tvec_ground, 0.2	)
        frame_with_markers_and_axis	=	cv2.aruco.drawAxis(	frame_with_markers_and_axis,  CamParam.camMatrix, CamParam.distCoeff, Rca_panel, tvec_panel, 0.2	)
        

        rvec_all_markers_ground, tvec_all_markers_ground, _ = aruco.estimatePoseSingleMarkers(corners[0:20], 0.04, CamParam.camMatrix, CamParam.distCoeff)
        rvec_all_markers_panel, tvec_all_markers_panel, _ = aruco.estimatePoseSingleMarkers(corners[20:45], 0.025, CamParam.camMatrix, CamParam.distCoeff)
        tvec_all=np.concatenate((tvec_all_markers_ground,tvec_all_markers_panel),axis=0)
        for i_ids,i_corners,i_tvec in zip(ids,corners,tvec_all):
            UV[i_ids,:] = i_corners.reshape([1,8]) #np.average(i_corners, axis=1) 
            P[i_ids,:] =i_tvec
        
        #print 'P',P
        #used to find the height of the tags and the delta change of height, z height at desired position
        Z = 1*P[32:48,2] #- [0.68184539, 0.68560932, 0.68966803, 0.69619578])

        #check if all tags detected
        if retVal_ground != 0 and retVal_panel !=0:
            dutmp = []
            dvtmp = []
            
            #Pixel estimates of the ideal ground tag location
            reprojected_imagePoints_ground_2, jacobian2	=	cv2.projectPoints(	loaded_object_points_ground_in_panel_system.transpose(), rvec_panel, tvec_panel, CamParam.camMatrix, CamParam.distCoeff)
            reprojected_imagePoints_ground_2=reprojected_imagePoints_ground_2.reshape([reprojected_imagePoints_ground_2.shape[0],2])
            
            #plot image points for ground tag from corner detection and from re-projections
            for point1,point2 in zip(imgPoints_ground,np.float32(reprojected_imagePoints_ground_2)):
                cv2.circle(frame_with_markers_and_axis,tuple(point1),5,(0,0,255),3)
                cv2.circle(frame_with_markers_and_axis,tuple(point2),5,(255,0,0),3) 
                
            
            cv2.imshow('Image',frame_with_markers_and_axis)
            cv2.waitKey(1)
            #Save
            filename_image = "/home/rpi-cats/Desktop/DJ/Code/Images/Panel1_Acquisition_"+str(t1)+"_"+str(iteration)+".jpg"
            scipy.misc.imsave(filename_image, frame_with_markers_and_axis)
            
            #Go through a particular point in all tags to build the complete Jacobian
            for ic in range(4):
                #uses first set of tags, numbers used to offset camera frame, come from camera parameters
                #UV_target = np.vstack([UV[9:13,2*ic]-1209.151959040735,UV[9:13,2*ic+1]-1055.254852652610]).T - UV_shift[:,(2*ic):(2*ic+2)]		#shift corner estimates to the image centers. Necessary for the image jacobian to work.
                reprojected_imagePoints_ground_2 = np.reshape(reprojected_imagePoints_ground_2,(16,8))
                UV_target = np.vstack([reprojected_imagePoints_ground_2[:,2*ic]-1192.255485086403,reprojected_imagePoints_ground_2[:,2*ic+1]-1021.399092147012]).T
                uc = UV_target[:,0]
                vc = UV_target[:,1]
    		    
#                print 'UV_target', UV_target
                UV_current = np.vstack([UV[32:48,2*ic]-1192.255485086403,UV[32:48,2*ic+1]-1021.399092147012]).T
                #find difference between current and desired tag difference
                delta_UV = UV_target-UV_current
                dutmp.append(np.mean(delta_UV[:,0]))
                dvtmp.append(np.mean(delta_UV[:,1]))
                for tag_i in range(16):
                    if tag_i == 0:
                        J_cam_tmp = 1.0*np.array([[-f_hat_u/Z[tag_i], 0.0, uc[tag_i]/Z[tag_i], uc[tag_i]*vc[tag_i]/f_hat_u, -1.0*(uc[tag_i]*uc[tag_i]/f_hat_u + f_hat_u), vc[tag_i]],
                                                   [0.0, -f_hat_v/Z[tag_i], vc[tag_i]/Z[tag_i], vc[tag_i]*vc[tag_i]/f_hat_v+f_hat_v, -1.0*uc[tag_i]*vc[tag_i]/f_hat_v, -uc[tag_i]]])
                    else:
                        J_cam_tmp= np.concatenate((J_cam_tmp, 1.0*np.array([[-f_hat_u/Z[tag_i], 0.0, uc[tag_i]/Z[tag_i], uc[tag_i]*vc[tag_i]/f_hat_u, -1.0*(uc[tag_i]*uc[tag_i]/f_hat_u + f_hat_u), vc[tag_i]],
                                                   [0.0, -f_hat_v/Z[tag_i], vc[tag_i]/Z[tag_i], vc[tag_i]*vc[tag_i]/f_hat_v+f_hat_v, -1.0*uc[tag_i]*vc[tag_i]/f_hat_v, -uc[tag_i]]])), axis=0)
                #camera jacobian
                if ic ==0:
                    J_cam = J_cam_tmp
                    delta_UV_all = delta_UV.reshape(32,1)
                    UV_target_all = UV_target.reshape(32,1)
                else:
                    J_cam = np.vstack([J_cam,J_cam_tmp])
                    delta_UV_all = np.vstack([delta_UV_all,delta_UV.reshape(32,1)]) 
                    UV_target_all = np.vstack([UV_target_all,UV_target.reshape(32,1)])

            print 'dutmp: ',dutmp
            print 'dvtmp: ',dvtmp            
            du = np.mean(np.absolute(dutmp))
            dv = np.mean(np.absolute(dvtmp))
            print 'Average du of all points:',du, 'Average dv of all points:',dv
            du_array.append(du)
            dv_array.append(dv)


            print "Jcam",J_cam 
            print "UV",delta_UV_all 
            dx = np.matmul(np.linalg.pinv(J_cam),np.array(delta_UV_all))
            dx_array.append(dx)

            dx = np.array([dx[3,0],-dx[4,0],-dx[5,0], dx[0,0],-dx[1,0],-dx[2,0]])

            if stage ==2:
                T = listener.lookupTransform("base", "link_6", rospy.Time(0))
                rg = 9.8*np.matmul(np.matmul(T.R,Tran_z).transpose(),np.array([0,0,1]).transpose())
                A1 = np.hstack([rox.hat(rg).transpose(),np.zeros([3,1]),np.eye(3),np.zeros([3,3])])
                A2 = np.hstack([np.zeros([3,3]),rg.reshape([3,1]),np.zeros([3,3]),np.eye(3)])
                A = np.vstack([A1,A2])
                FTdata_est = np.matmul(A,Vec_wrench)
                FTread = FTdata-FTdata_0-FTdata_est+FTdata_0est
                print 'FTread:',FTread
        
                print 'Z',FTread[-1]
                if FTread[-1]>-100:
                    F_d = -150
                    Kc = 0.0001    
                else:
                    Kc = 0.0001
                    F_d = -200

                Vz = Kc*(F_d - FTread[-1])
                print 'Vz',Vz
                dx[-1] = Vz+dx[-1]
                FTread_array.append(FTread)
                FTdata_array.append(FTdata)				
				

            current_joint_angles = controller_commander.get_current_joint_values()


            step_size_tmp = np.linalg.norm(dx)
            if step_size_tmp <= step_size_min:
                step_size_min = step_size_tmp
            else:
                dx = dx/step_size_tmp*step_size_min


            joints_vel = QP_abbirb6640(np.array(current_joint_angles).reshape(6, 1),np.array(dx))
 
            goal = trapezoid_gen(np.array(current_joint_angles) + joints_vel.dot(dt),np.array(current_joint_angles),0.25,np.array(dx))
            step_size.append(np.linalg.norm(dx))


            client = actionlib.SimpleActionClient("joint_trajectory_action", FollowJointTrajectoryAction)
            client.wait_for_server()
    
            client.send_goal(goal)
            client.wait_for_result()
            res = client.get_result()
            if (res.error_code != 0):
                raise Exception("Trajectory execution returned error")
    
            print res

            
            print 'Current Iteration Finished.'
            
     

    #Saving iteration data to file
    filename_data = "/home/rpi-cats/Desktop/YC/Data/Panel1_Data_"+str(t1)+".mat"
    scipy.io.savemat(filename_data, mdict={'du_array':du_array, 'dv_array':dv_array, 'dx_array':dx_array, 'step_size':step_size, 'iteration':iteration})
	
	
    #Saving force data to file
    filename_force_data = "/home/rpi-cats/Desktop/YC/Data/Panel1_Data_force_"+str(t1)+".mat"
    scipy.io.savemat(filename_force_data, mdict={'FTread':FTread_array, 'FTdata':FTdata_array, 't_now_array':t_now_array})	
    
    print '###############################'
    print 'step_size',step_size
    print 'iteration',iteration
    print '###############################'

    '''
    print "============ Final Push Down to Nest"
    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.2, [],[])

    
    Cur_Pose = controller_commander.get_current_pose_msg()
    rot_current = [Cur_Pose.pose.orientation.w, Cur_Pose.pose.orientation.x,Cur_Pose.pose.orientation.y,Cur_Pose.pose.orientation.z]
    trans_current = [Cur_Pose.pose.position.x,Cur_Pose.pose.position.y,Cur_Pose.pose.position.z]
    pose_target2.R = rox.q2R([rot_current[0], rot_current[1], rot_current[2], rot_current[3]])
    pose_target2.p = trans_current
    pose_target2.p[2] -= 0.11
    '''



    #### Final Nest Placement Error Calculation ===============================
    #Read new image
    last_ros_image_stamp = object_commander.ros_image_stamp        
    try:
        ros_gripper_2_trigger.wait_for_service(timeout=0.1)
        ros_gripper_2_trigger(False)
    except:
        pass
    wait_count=0
    while object_commander.ros_image is None or object_commander.ros_image_stamp == last_ros_image_stamp:
        if wait_count > 50:
            raise Exception("Image receive timeout")
        time.sleep(0.25)
        wait_count += 1
    result = object_commander.ros_image    
    
    #Detect tag corners in aqcuired image using aruco
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(result, aruco_dict, parameters=parameters)
    
    #Sort corners and ids according to ascending order of ids
    corners_original=copy.deepcopy(corners)
    ids_original=np.copy(ids)
    sorting_indices=np.argsort(ids_original,0)
    ids_sorted=ids_original[sorting_indices]
    ids_sorted=ids_sorted.reshape([len(ids_original),1])
    combined_list=zip(np.ndarray.tolist(ids.flatten()),corners_original)
    combined_list.sort()
    corners_sorted=[x for y,x in combined_list]
    ids=np.copy(ids_sorted)
    corners=copy.deepcopy(corners_sorted)
    
    #Remove ids and corresponsing corners not in range (Parasitic detections in random locations in the image)
    #false_ids_ind = np.where(ids>73)       
    mask = np.ones(ids.shape, dtype=bool)        
    #mask[false_ids_ind] = False
    ids = ids[mask]        
    corners = np.array(corners)        
    corners = corners[mask.flatten(),:]        
    corners = list(corners)
    
    #Define object and image points of both tags        
    objPoints_ground, imgPoints_ground	=	aruco.getBoardObjectAndImagePoints(board_ground, corners, ids)
    objPoints_panel, imgPoints_panel	=	aruco.getBoardObjectAndImagePoints(board_panel, corners, ids)
    objPoints_ground=objPoints_ground.reshape([objPoints_ground.shape[0],3])
    imgPoints_ground=imgPoints_ground.reshape([imgPoints_ground.shape[0],2])
    objPoints_panel=objPoints_panel.reshape([objPoints_panel.shape[0],3])
    imgPoints_panel=imgPoints_panel.reshape([imgPoints_panel.shape[0],2]) 

    #Save pose of marker boards after the iterations end(while in the final hovering position above nest)
    #Get pose of both ground and panel markers from detected corners        
    retVal_ground, rvec_ground, tvec_ground = cv2.solvePnP(objPoints_ground, imgPoints_ground, CamParam.camMatrix, CamParam.distCoeff)
    Rca_ground, b_ground = cv2.Rodrigues(rvec_ground)
    retVal_panel, rvec_panel, tvec_panel = cv2.solvePnP(objPoints_panel, imgPoints_panel, CamParam.camMatrix, CamParam.distCoeff)
    Rca_panel, b_panel = cv2.Rodrigues(rvec_panel)
    
    print"============== Observed Pose difference in nest position"
    observed_tvec_difference=tvec_ground-tvec_panel
    observed_rvec_difference=rvec_ground-rvec_panel
    print "tvec difference: ",observed_tvec_difference
    print "rvec differnece: ",observed_rvec_difference
    
    #Load ideal pose differnece information from file
    loaded_rvec_difference = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['rvec_difference']
    loaded_tvec_difference = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['tvec_difference']

    print"============== Ideal Pose difference in nest position"
    print "tvec difference: ",loaded_tvec_difference
    print "rvec differnece: ",loaded_rvec_difference

    print"============== Difference in pose difference in nest position"
    tvec_err = loaded_tvec_difference-observed_tvec_difference
    rvec_err = loaded_rvec_difference-observed_rvec_difference 
    print "tvec difference: ",tvec_err
    print "rvec differnece: ",rvec_err
    

	
    # Adjustment
    print "Adjustment ===================="
    current_joint_angles = controller_commander.get_current_joint_values()
    dx = np.array([0,0,0, -tvec_err[0], tvec_err[1],0])
    joints_vel = QP_abbirb6640(np.array(current_joint_angles).reshape(6, 1),np.array(dx))
    goal = trapezoid_gen(np.array(current_joint_angles) + joints_vel.dot(1),np.array(current_joint_angles),0.25,np.array(dx))
    client = actionlib.SimpleActionClient("joint_trajectory_action", FollowJointTrajectoryAction)
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result()
    res = client.get_result()
    if (res.error_code != 0):
        raise Exception("Trajectory execution returned error")

    print res
    print "End of Adjustment ===================="

	
    #### Final Nest Placement Error Calculation ===============================
    #Read new image
    last_ros_image_stamp = object_commander.ros_image_stamp        
    try:
        ros_gripper_2_trigger.wait_for_service(timeout=0.1)
        ros_gripper_2_trigger(False)
    except:
        pass
    wait_count=0
    while object_commander.ros_image is None or object_commander.ros_image_stamp == last_ros_image_stamp:
        if wait_count > 50:
            raise Exception("Image receive timeout")
        time.sleep(0.25)
        wait_count += 1
    result = object_commander.ros_image    
    
    #Detect tag corners in aqcuired image using aruco
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(result, aruco_dict, parameters=parameters)
    
    #Sort corners and ids according to ascending order of ids
    corners_original=copy.deepcopy(corners)
    ids_original=np.copy(ids)
    sorting_indices=np.argsort(ids_original,0)
    ids_sorted=ids_original[sorting_indices]
    ids_sorted=ids_sorted.reshape([len(ids_original),1])
    combined_list=zip(np.ndarray.tolist(ids.flatten()),corners_original)
    combined_list.sort()
    corners_sorted=[x for y,x in combined_list]
    ids=np.copy(ids_sorted)
    corners=copy.deepcopy(corners_sorted)
    
    #Remove ids and corresponsing corners not in range (Parasitic detections in random locations in the image)
    #false_ids_ind = np.where(ids>73)       
    mask = np.ones(ids.shape, dtype=bool)        
    #mask[false_ids_ind] = False
    ids = ids[mask]        
    corners = np.array(corners)        
    corners = corners[mask.flatten(),:]        
    corners = list(corners)
    
    #Define object and image points of both tags        
    objPoints_ground, imgPoints_ground	=	aruco.getBoardObjectAndImagePoints(board_ground, corners, ids)
    objPoints_panel, imgPoints_panel	=	aruco.getBoardObjectAndImagePoints(board_panel, corners, ids)
    objPoints_ground=objPoints_ground.reshape([objPoints_ground.shape[0],3])
    imgPoints_ground=imgPoints_ground.reshape([imgPoints_ground.shape[0],2])
    objPoints_panel=objPoints_panel.reshape([objPoints_panel.shape[0],3])
    imgPoints_panel=imgPoints_panel.reshape([imgPoints_panel.shape[0],2]) 

    #Save pose of marker boards after the iterations end(while in the final hovering position above nest)
    #Get pose of both ground and panel markers from detected corners        
    retVal_ground, rvec_ground, tvec_ground = cv2.solvePnP(objPoints_ground, imgPoints_ground, CamParam.camMatrix, CamParam.distCoeff)
    Rca_ground, b_ground = cv2.Rodrigues(rvec_ground)
    retVal_panel, rvec_panel, tvec_panel = cv2.solvePnP(objPoints_panel, imgPoints_panel, CamParam.camMatrix, CamParam.distCoeff)
    Rca_panel, b_panel = cv2.Rodrigues(rvec_panel)
    
    print"============== Observed Pose difference in nest position"
    observed_tvec_difference=tvec_ground-tvec_panel
    observed_rvec_difference=rvec_ground-rvec_panel
    print "tvec difference: ",observed_tvec_difference
    print "rvec differnece: ",observed_rvec_difference
    
    #Load ideal pose differnece information from file
    loaded_rvec_difference = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['rvec_difference']
    loaded_tvec_difference = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['tvec_difference']

    print"============== Ideal Pose difference in nest position"
    print "tvec difference: ",loaded_tvec_difference
    print "rvec differnece: ",loaded_rvec_difference

    print"============== Difference in pose difference in nest position"
    tvec_err = loaded_tvec_difference-observed_tvec_difference
    rvec_err = loaded_rvec_difference-observed_rvec_difference 
    print "tvec difference: ",tvec_err
    print "rvec differnece: ",loaded_rvec_difference-observed_rvec_difference 
    
    #Saving pose information to file
    filename_pose2 = "/home/rpi-cats/Desktop/DJ/Code/Data/In_Nest_Pose_"+str(t1)+".mat"
    scipy.io.savemat(filename_pose2, mdict={'tvec_ground_In_Nest':tvec_ground, 'tvec_panel_In_Nest':tvec_panel, 
    'Rca_ground_In_Nest':Rca_ground, 'Rca_panel_In_Nest': Rca_panel, 'tvec_difference_In_Nest': tvec_ground-tvec_panel, 'rvec_difference_In_Nest': rvec_ground-rvec_panel, 
    'loaded_tvec_difference':loaded_tvec_difference, 'loaded_rvec_difference':loaded_rvec_difference,'observed_tvec_difference':observed_tvec_difference,'observed_rvec_difference':observed_rvec_difference})	
    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.7, [])
#    raw_input("confirm release vacuum")
    rapid_node.set_digital_io("Vacuum_enable", 0)
    g=ProcessStepGoal('plan_place_set_second_step',"")


    process_client.send_goal(g)

    #self.in_process=True

    print process_client.get_result()
    print "VACUUM OFF!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    time.sleep(0.5)

    tran = np.array([2.17209718963,-1.13702651252,0.224273328841])
    rot = rox.q2R([0.0145063655084, -0.0282932350764, 0.999322921073, -0.0185137145776])
    pose_target2 = rox.Transform(rot, tran)

    Cur_Pose = controller_commander.get_current_pose_msg()
    rot_current = [Cur_Pose.pose.orientation.w, Cur_Pose.pose.orientation.x,Cur_Pose.pose.orientation.y,Cur_Pose.pose.orientation.z]
    trans_current = [Cur_Pose.pose.position.x,Cur_Pose.pose.position.y,Cur_Pose.pose.position.z]
    pose_target2.R = rox.q2R([rot_current[0], rot_current[1], rot_current[2], rot_current[3]])
    pose_target2.p = trans_current
    pose_target2.p[2] += 0.5
#
#
#    #print 'Target:',pose_target3
#
#    #print "============ Executing plan4"
    controller_commander.compute_cartesian_path_and_move(pose_target2, avoid_collisions=False)


    t2 = time.time()
    print 'Execution Finished.'
    print "Execution time: " + str(t2-t1) + " seconds"

if __name__ == '__main__':
    main()
