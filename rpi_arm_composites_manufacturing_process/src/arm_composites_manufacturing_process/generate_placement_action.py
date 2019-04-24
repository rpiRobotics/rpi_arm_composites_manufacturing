#!/usr/bin/env python

#from .arm_composites_manufacturing_placement import PlacementController
import rospy
import time
import actionlib
import cv2
import cv2.aruco as aruco
from geometry_msgs.msg import Pose, Point, Vector3
from ibvs_object_placement.msg import PlacementStepAction, PlacementStepGoal, PlacementCommand, IBVSParameters, ComplianceControlParameters
from scipy.io import loadmat
from industrial_payload_manager.msg import ArucoGridboard
import rospkg
import os

def _finished_client(self,state,result):
    print "feedback received"
    

def _placement(panel_type,done_cb=None):
    #rospy.init_node("pick_and_place_panel", anonymous=True)
    #placementclient=actionlib.ActionClient('placement_step', PlacementStepAction)
    #placementclient.wait_for_server()
    #time.sleep(0.5)
    placement_command=PlacementCommand()
    compliance_control=ComplianceControlParameters()
    ibvs_params=IBVSParameters()
    #make sure command.cameras
    placement_command.cameras=["gripper_camera_2"]
    aruco_dict ="DICT_ARUCO_ORIGINAL"
    compliance_control.F_d_set1=-200
    compliance_control.F_d_set2=-360
    compliance_control.Kc=0.00025
    placement_command.compliance_control_parameters=compliance_control
    ibvs_params.IBVSdt=0.1
    ibvs_params.du_converge_th= 3
    ibvs_params.dv_converge_th=3
    ibvs_params.iteration_limit=70
    ibvs_params.Ki=1.0
    ibvs_params.step_size_min=100000
    placement_command.ibvs_parameters=ibvs_params
    placement_goal=PlacementStepGoal()
    def wrap_vector3(data):
        vector=Vector3()
        vector.x=data[0]
        vector.y=data[1]
        vector.z=data[2]
        return vector
        
        
    def wrap_points(point_diff):
        point_array=[]
        num_points=point_diff.shape[1]
        for i in range(num_points):
            point=Point()
            point.x=point_diff[0][i]
            point.y=point_diff[1][i]
            point.z=point_diff[2][i]
            point_array.append(point)
        return point_array
    
    if(panel_type=='leeward_mid_panel'):
        placement_command.rvec_difference_stage1=wrap_vector3(loadmat(os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_process'), 'config/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_Above_Nest.mat'))['rvec_difference'])
        placement_command.tvec_difference_stage1=wrap_vector3( loadmat(os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_process'),'config/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_Above_Nest.mat'))['tvec_difference'])
        placement_command.rvec_difference_stage2=wrap_vector3(loadmat(os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_process'),'config/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat'))['rvec_difference'])
        placement_command.tvec_difference_stage2=wrap_vector3( loadmat(os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_process'),'config/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat'))['tvec_difference'])
        placement_command.point_difference_stage2=wrap_points(loadmat(os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_process'),'config/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat'))['object_points_ground_in_panel_tag_system'])
        initial_place=Pose()
        initial_place.position.x=2.15484
        initial_place.position.y=1.21372
        initial_place.position.z=0.25766
        initial_place.orientation.w=0.02110
        initial_place.orientation.x= -0.03317
        initial_place.orientation.y= 0.99922
        initial_place.orientation.z=-0.00468
        placement_command.initial=initial_place
        #CHECK THESE AGAIN
        placement_goal.camera1ground=ArucoGridboard(4,4,0.04,0.0075,aruco_dict,32)
        placement_goal.camera1place=ArucoGridboard(8,3,0.025,0.0075,aruco_dict,80)
    elif(panel_type=='leeward_tip_panel'):
        placement_command.rvec_difference_stage1=wrap_vector3(loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel2_Cam_636_object_points_ground_tag_in_panel_frame_Offset_In_Nest.mat')['rvec_difference'])
        placement_command.tvec_difference_stage1=wrap_vector3(loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel2_Cam_636_object_points_ground_tag_in_panel_frame_Offset_In_Nest.mat')['tvec_difference'])
        placement_command.rvec_difference_stage2=wrap_vector3(loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel2_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['rvec_difference'])
        placement_command.tvec_difference_stage2=wrap_vector3( loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel2_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['tvec_difference'])
        placement_command.point_difference_stage2=wrap_points(loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel2_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['object_points_ground_in_panel_tag_system'])
        initial_place=Pose()     
        initial_place.position.x=2.20775354978
        initial_place.position.y=-1.08835759918
        initial_place.position.z=0.467008345754
        initial_place.orientation.w=0.0360399909458
        initial_place.orientation.x=-0.0155318641364
        initial_place.orientation.y= 0.999181449999
        initial_place.orientation.z=-0.00981377740413
        placement_command.initial=initial_place
        #CHECK THESE AGAIN
        placement_goal.camera1ground=ArucoGridboard(4,4,0.04,0.0075,aruco_dict,16)
        placement_goal.camera1place=ArucoGridboard(8,3,0.025,0.0075,aruco_dict,50)
        
        
    placement_goal.data=placement_command
    return placement_goal
    #client_handle=placementclient.send_goal(placement_goal,done_cb=done_cb)
        
    
    rospy.loginfo("End panel pick and place script")

if __name__ == '__main__':
    _placement('leeward_mid_panel',_finished_client)
