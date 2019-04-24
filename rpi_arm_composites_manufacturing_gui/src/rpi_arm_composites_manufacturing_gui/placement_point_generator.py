import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from pyspin_wrapper.srv import CameraTrigger
from cv_bridge import CvBridge, CvBridgeError
import time
import sys
from scipy.linalg import expm
import cv2
import cv2.aruco as aruco
import scipy.misc
from industrial_payload_manager import PayloadTransformListener
from scipy.io import loadmat
import actionlib
import copy
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal

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


def main():
    t1 = time.time()
    rospy.init_node('Placement_DJ_1', anonymous=True)
    image_pub=rospy.Publisher('/gripper_camera_1/image',Image,queue_size=10)
    object_commander=ObjectRecognitionCommander()
    ros_gripper_2_img_sub=rospy.Subscriber('/gripper_camera_2/image', Image, object_commander.ros_raw_gripper_2_image_cb)
    ros_gripper_2_trigger=rospy.ServiceProxy('/gripper_camera_2/continuous_trigger', CameraTrigger)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters =  cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.adaptiveThreshWinSizeMax=30
    parameters.adaptiveThreshWinSizeStep=7
    last_ros_image_stamp=rospy.Time(0)
    placing_panel_2=False
    
    board_ground = cv2.aruco.GridBoard_create(4, 4, .04, .0075, aruco_dict, 32)
    board_panel = cv2.aruco.GridBoard_create(8, 3, .025, .0075, aruco_dict, 80)
    board_mid_panel_2=cv2.aruco.GridBoard_create(4, 4, .04, .0075, aruco_dict, 16)
    board_tip_panel=cv2.aruco.GridBoard_create(8, 3, .025, .0075, aruco_dict, 50)
    CamParam = CameraParams(2283.391289766133, 1192.255485086403, 2279.484382094687, 1021.399092147012, 1.0, -0.022101211408596, -0.095163053709332, -0.003986991791212,  -0.003479613658352, 0.179926705467534)
    UV = np.zeros([150,8])
    P = np.zeros([150,3])
    #loaded_object_points_ground_in_panel_system_stage_1 = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_Above_Nest.mat') ['object_points_ground_in_panel_tag_system']
    loaded_object_points_ground_in_panel_system_stage_2 = loadmat('/home/rpi-cats/Desktop/YC/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['object_points_ground_in_panel_tag_system']   
    loaded_rvec_difference = loadmat('/home/rpi-cats/Desktop/YC/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['rvec_difference']
    loaded_tvec_difference = loadmat('/home/rpi-cats/Desktop/YC/Panel1_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['tvec_difference']    
    loaded_object_points_ground_in_panel_system = loaded_object_points_ground_in_panel_system_stage_2
    f_hat_u = 2283.391289766133#2342.561249990927#2282.523358266698#2281.339593273153 #2446.88
    f_hat_v = 2279.484382094687#2338.448312671424#2280.155828279608
    du_array=[]
    dv_array=[]
    dx_array=[]
    cv2.namedWindow('Image',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Image', 2000,2000)
    last_ros_image_stamp = object_commander.ros_image_stamp 
    try:
        ros_gripper_2_trigger.wait_for_service(timeout=0.1)
        ros_gripper_2_trigger(False)
        print "triggered"
    except:
        pass
    
    wait_count=0
    while object_commander.ros_image is None or object_commander.ros_image_stamp == last_ros_image_stamp:
        if wait_count > 50:
            raise Exception("Image receive timeout")
        time.sleep(0.25)
        wait_count += 1
    result = object_commander.ros_image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(result, aruco_dict, parameters=parameters)
    if(20 in ids):
        print "Now placing panel 2"
        placing_panel_2=True
    	board_ground=board_mid_panel_2
    	board_panel=board_tip_panel
        loaded_object_points_ground_in_panel_system_stage_1 = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel2_Cam_636_object_points_ground_tag_in_panel_frame_Offset_In_Nest.mat') ['object_points_ground_in_panel_tag_system']
        loaded_object_points_ground_in_panel_system_stage_2 = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel2_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['object_points_ground_in_panel_tag_system']
        loaded_rvec_difference = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel2_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['rvec_difference']
        loaded_tvec_difference = loadmat('/home/rpi-cats/Desktop/DJ/Ideal Position/Panel2_Cam_636_object_points_ground_tag_in_panel_frame_In_Nest.mat')['tvec_difference']
  
        loaded_object_points_ground_in_panel_system = loaded_object_points_ground_in_panel_system_stage_2


    while(True):

        last_ros_image_stamp = object_commander.ros_image_stamp 
        try:
            ros_gripper_2_trigger.wait_for_service(timeout=0.1)
            ros_gripper_2_trigger(False)
            print "triggered"
        except:
            pass
        
        wait_count=0
        while object_commander.ros_image is None or object_commander.ros_image_stamp == last_ros_image_stamp:
            if wait_count > 50:
                raise Exception("Image receive timeout")
            time.sleep(0.25)
            wait_count += 1
        result = object_commander.ros_image
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(result, aruco_dict, parameters=parameters)

        frame_with_markers_and_axis=result
        corners_original=copy.deepcopy(corners)
        ids_original=np.copy(ids)
        sorting_indices=np.argsort(ids_original,0)
        ids_sorted=ids_original[sorting_indices]
        ids_sorted=ids_sorted.reshape([len(ids_original),1])
        combined_list=zip(np.ndarray.tolist(ids.flatten()),corners_original)
    #        print "combined_list:",combined_list
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
        #print "sorted ids: ",ids
        #print "sorted corners: ",corners
        
        objPoints_ground, imgPoints_ground	=	aruco.getBoardObjectAndImagePoints(board_ground, corners, ids)
        objPoints_panel, imgPoints_panel	=	aruco.getBoardObjectAndImagePoints(board_panel, corners, ids)
        objPoints_ground=objPoints_ground.reshape([objPoints_ground.shape[0],3])
        imgPoints_ground=imgPoints_ground.reshape([imgPoints_ground.shape[0],2])
        objPoints_panel=objPoints_panel.reshape([objPoints_panel.shape[0],3])
        imgPoints_panel=imgPoints_panel.reshape([imgPoints_panel.shape[0],2])
        #objPoints_ground, imgPoints_ground	=	aruco.getBoardObjectAndImagePoints(board_ground, corners, ids)
               
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
            #print 'i_corners',i_corners,i_corners.reshape([1,8])
            UV[i_ids,:] = i_corners.reshape([1,8]) #np.average(i_corners, axis=1) 
            P[i_ids,:] =i_tvec
        
        
        #used to find the height of the tags and the delta change of height, z height at desired position
        '''
        Z = 1*P[32:48,2] #- [0.68184539, 0.68560932, 0.68966803, 0.69619578])
        if(placing_panel_2):
            Z = 1*P[16:32,2]
        '''
        #check if all tags detected
        if retVal_ground != 0 and retVal_panel !=0:
            dutmp = []
            dvtmp = []
            
            #Pixel estimates of the ideal ground tag location
            reprojected_imagePoints_ground_2, jacobian2	=	cv2.projectPoints(	loaded_object_points_ground_in_panel_system.transpose(), rvec_panel, tvec_panel, CamParam.camMatrix, CamParam.distCoeff)
            reprojected_imagePoints_ground_2 = reprojected_imagePoints_ground_2.reshape([reprojected_imagePoints_ground_2.shape[0],2])
           
            
            #plot image points for ground tag from corner detection and from re-projections
            for point1,point2 in zip(imgPoints_ground,np.float32(reprojected_imagePoints_ground_2)):
                cv2.circle(frame_with_markers_and_axis,tuple(point1),5,(0,0,255),3)
                cv2.circle(frame_with_markers_and_axis,tuple(point2),5,(255,0,0),3)  

            #print 'Test:', imgPoints_ground-np.float32(reprojected_imagePoints_ground_2)
            tmp = imgPoints_ground-np.float32(reprojected_imagePoints_ground_2)
    
            cv2.imshow('Image',frame_with_markers_and_axis)
            cv2.waitKey(1)
            image_msg=object_commander.bridge.cv2_to_imgmsg(frame_with_markers_and_axis,"mono8")
            image_msg.header.stamp=rospy.Time.now()
            image_pub.publish(image_msg)


            '''                
            for ic in range(4):
                #uses first set of tags, numbers used to offset camera frame, come from camera parameters
                #UV_target = np.vstack([UV[9:13,2*ic]-1209.151959040735,UV[9:13,2*ic+1]-1055.254852652610]).T - UV_shift[:,(2*ic):(2*ic+2)]		#shift corner estimates to the image centers. Necessary for the image jacobian to work.
                reprojected_imagePoints_ground_2 = np.reshape(reprojected_imagePoints_ground_2,(16,8))
                UV_target = np.vstack([reprojected_imagePoints_ground_2[:,2*ic]-1192.255485086403,reprojected_imagePoints_ground_2[:,2*ic+1]-1021.399092147012]).T
                uc = UV_target[:,0]
                vc = UV_target[:,1]
    		    
#                print 'UV_target', UV_target
                UV_current = np.vstack([UV[16:32,2*ic]-1192.255485086403,UV[16:32,2*ic+1]-1021.399092147012]).T
                #find difference between current and desired tag difference
                delta_UV = UV_target-UV_current
#                print 'delta_UV: ',ic, delta_UV
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
            '''

            #tmp = imgPoints_ground-np.float32(reprojected_imagePoints_ground_2)
            dutmp = tmp[:,0]
            dvtmp = tmp[:,1]
            print 'dutmp: ',dutmp
            print 'dvtmp: ',dvtmp            
            du = np.mean(np.absolute(dutmp))
            dv = np.mean(np.absolute(dvtmp))    
            print 'Average du of all points:',du, 'Average dv of all points:',dv    
    


    
        #print"============== Observed Pose difference in nest position"
        observed_tvec_difference=tvec_ground-tvec_panel
        observed_rvec_difference=rvec_ground-rvec_panel
        #print "tvec difference: ",observed_tvec_difference
        #print "rvec differnece: ",observed_rvec_difference
        
        #Load ideal pose differnece information from file
      

        #print"============== Ideal Pose difference in nest position"
        #print "tvec difference: ",loaded_tvec_difference
        #print "rvec differnece: ",loaded_rvec_difference

        print"============== Difference in pose difference in nest position"
        tvec_err = loaded_tvec_difference-observed_tvec_difference
        rvec_err = loaded_rvec_difference-observed_rvec_difference 
        print "tvec difference: ",tvec_err
        print "rvec differnece: ",rvec_err


        filename_pose2 = "/home/rpi-cats/KPPplacementdata/Panel2_In_Nest_Pose_"+str(t1)+".mat"
        scipy.io.savemat(filename_pose2, mdict={'tvec_ground_In_Nest':tvec_ground, 'tvec_panel_In_Nest':tvec_panel, 
    'Rca_ground_In_Nest':Rca_ground, 'Rca_panel_In_Nest': Rca_panel, 'tvec_difference_In_Nest': tvec_ground-tvec_panel, 'rvec_difference_In_Nest': rvec_ground-rvec_panel, 
    'loaded_tvec_difference':loaded_tvec_difference, 'loaded_rvec_difference':loaded_rvec_difference,'observed_tvec_difference':observed_tvec_difference,'observed_rvec_difference':observed_rvec_difference,'du':du, 'dv':dv})	 
        
if __name__ == '__main__':
    main()
    
