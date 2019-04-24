# Copyright (c) 2018, Rensselaer Polytechnic Institute, Wason Technology LLC
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Rensselaer Polytechnic Institute, nor Wason 
#       Technology LLC, nor the names of its contributors may be used to 
#       endorse or promote products derived from this software without 
#       specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import copy
import rospy
import actionlib
import rospkg
import general_robotics_toolbox as rox
import general_robotics_toolbox.urdf as urdf
import general_robotics_toolbox.ros_msg as rox_msg
from general_robotics_toolbox import ros_tf as tf
import yaml
import genpy

import rpi_abb_irc5.ros.rapid_commander as rapid_node_pkg
from safe_kinematic_controller.ros.commander import ControllerCommander
from rpi_arm_composites_manufacturing_process.msg import ProcessState, ProcessStepResult
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal

from industrial_payload_manager.payload_transform_listener import PayloadTransformListener
from industrial_payload_manager.srv import UpdatePayloadPose, UpdatePayloadPoseRequest, \
    GetPayloadArray, GetPayloadArrayRequest
import time
import sys
import os
from geometry_msgs.msg import Wrench, Vector3, TransformStamped
from pbvs_object_placement.msg import PBVSPlacementAction, PBVSPlacementGoal
import threading
import traceback
import resource_retriever
import urlparse
from urdf_parser_py.urdf import URDF
from tf.msg import tfMessage
from visualization_msgs.msg import Marker, MarkerArray
import subprocess
from .planner import Planner
from .generate_placement_action import _placement



class ProcessController(object):
    def __init__(self, disable_ft=False):
        self.lock=threading.Lock()
        urdf_xml_string = rospy.get_param("robot_description")
        srdf_xml_string = rospy.get_param("robot_description_semantic")
        self.urdf=URDF.from_parameter_server()
        self.overhead_vision_client=actionlib.SimpleActionClient("recognize_objects", ObjectRecognitionAction)        
        self.rapid_node = rapid_node_pkg.RAPIDCommander()
        self.controller_commander=ControllerCommander()
        self.state='init'
        self.current_target=None
        self.current_payload=None
        self.available_payloads={'leeward_mid_panel': 'leeward_mid_panel','leeward_tip_panel':'leeward_tip_panel'}
        self.desired_controller_mode=self.controller_commander.MODE_AUTO_TRAJECTORY
        self.speed_scalar=1.0
        self.disable_ft=disable_ft        
        
        self.tf_listener=PayloadTransformListener()
        self._process_state_pub = rospy.Publisher("process_state", ProcessState, queue_size=100, latch=True)
        self.publish_process_state()
        self.placement_client=actionlib.SimpleActionClient('placement_step', PBVSPlacementAction)
        #self.placement_client.wait_for_server()
        self.update_payload_pose_srv=rospy.ServiceProxy("update_payload_pose", UpdatePayloadPose)
        self.get_payload_array_srv=rospy.ServiceProxy("get_payload_array", GetPayloadArray)
        self._goal_handle=None
        self._goal_handle_lock=threading.RLock()
        self.subprocess_handle=None
        self.plan_dictionary={}
        self.process_starts={}
        self.process_index=None
        self.process_states=["reset_position","pickup_prepare","pickup_lower","pickup_grab_first_step","pickup_grab_second_step","pickup_raise","transport_payload","place_payload","gripper_release"]
        
        self.planner=Planner(self.controller_commander, urdf_xml_string, srdf_xml_string)
        
        
    def _vision_get_object_pose(self, key):
        self.overhead_vision_client.wait_for_server()
        
        goal=ObjectRecognitionGoal(False, [-1e10,1e10,-1e10,1e10])
        self.overhead_vision_client.send_goal(goal)
        self.overhead_vision_client.wait_for_result()
        ret=self.overhead_vision_client.get_result()
                        
        for r in ret.recognized_objects.objects:
            if r.type.key == key:
                rox_pose=rox_msg.msg2transform(r.pose.pose.pose)
                rox_pose.parent_frame_id=r.pose.header.frame_id
                rox_pose.child_frame_id=key
                return rox_pose
            
        raise Exception("Requested object not found")
    
    def _vision_get_object_gripper_target_pose(self, key):
        
        object_pose=self._vision_get_object_pose(key)
                
        tag_rel_pose = self.tf_listener.lookupTransform(key, key + "_gripper_target", rospy.Time(0))        
        return object_pose * tag_rel_pose, object_pose
    
    def _tf_get_object_gripper_target_pose(self, key):
        
        payload=self._get_payload(key)
        if payload.confidence < 0.8:
            raise Exception("Payload confidence too low for tf lookup")
        
        object_pose = self.tf_listener.lookupTransform("/world", key, rospy.Time(0))
                
        tag_rel_pose = self.tf_listener.lookupTransform(key, key + "_gripper_target", rospy.Time(0))        
        return object_pose * tag_rel_pose, object_pose
    
    
    
    def get_payload_pickup_ft_threshold(self, payload):
        if self.disable_ft:
            return []
        return self._get_payload(payload).gripper_targets[0].pickup_ft_threshold
    
    def get_state(self):
        return self.state
    
    def get_current_pose(self):
        return self.controller_commander.get_current_pose_msg()
        
    def cancel_step(self, goal=None):        
        with self._goal_handle_lock:
            self.stop_motion()
               
    def stop_motion(self):
        with self._goal_handle_lock:
            self.controller_commander.stop_trajectory()
            
    def plan_rewind_motion(self, goal):
        no_rewind_list=[None,0,4,5,8]
        self._begin_step(goal)
        if(self.process_index not in no_rewind_list):
            try:
                if(self.process_index==3):
                    self.process_index-=1
                    rewind_target_pose=self.process_starts[self.process_states[self.process_index]]
                    
                else:
                    rewind_target_pose=self.process_starts[self.process_states[self.process_index]]
                path=self._plan(rewind_target_pose, config = "reposition_robot", smoother_config = "reposition_robot_smoother")
                self.plan_dictionary['rewind_motion']=path
                self._step_complete(goal)
            except Exception as err:
                traceback.print_exc()
                self._step_failed(err, goal)
            
        else:
            self._step_failed("Rewind Unavailable, Please Manually Reposition Robot",goal)
            
            
    def move_rewind_motion(self,mode,goal):
        self._begin_step(goal)
        try:
            self.controller_commander.set_controller_mode(mode, self.speed_scalar,[], [])
            path=self.plan_dictionary['rewind_motion']
            
            self._execute_path(path, goal)
            self.process_index-=1
            self.state=self.process_states[self.process_index]
                
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)
            
    
    def plan_reset_position(self, goal = None):
        
        self._begin_step(goal)
        try:            
            Q=[0.02196692, -0.10959773,  0.99369529, -0.00868731]
            P=[1.8475985 , -0.04983688,  0.82486047]
                    
            rospy.loginfo("Planning to reset position")
            self.state="reset_position"
            self.process_index=0
            
            pose_target=rox.Transform(rox.q2R(Q), np.copy(P))
            
            path=self._plan(pose_target, config = "reposition_robot", smoother_config = "reposition_robot_smoother")
            
            self.plan_dictionary['reset_position']=path
            
            self._step_complete(goal)
            
            
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)
            
    def move_reset_position(self, mode = ControllerCommander.MODE_AUTO_TRAJECTORY, goal = None):
        
        self._begin_step(goal)
        try:            
            
            
            self.controller_commander.set_controller_mode(self.controller_commander.MODE_HALT, self.speed_scalar,[], [])
            self.controller_commander.set_controller_mode(mode, self.speed_scalar,[], [])
            path=self.plan_dictionary['reset_position']      
            self._execute_path(path, goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)
            	
    def place_panel(self, target_payload, goal = None):
        self._begin_step(goal)
        
            
                
        def done_cb(status,result):
            rospy.loginfo("ibvs placement generated success")
            if(goal is not None):                
                if status == actionlib.GoalStatus.SUCCEEDED:
                    self._step_complete(goal)
                else:
                    with self._goal_handle_lock:
                        if self._goal_handle == goal:
                            self._goal_handle = None
                    res = ProcessStepResult()
                    res.state=self.state
                    res.target=self.current_target if self.current_target is not None else ""
                    res.payload=self.current_payload if self.current_payload is not None else ""
                    res.error_msg=str(result.error_msg)    
                    goal.set_aborted(result=res)                    
                    rospy.loginfo("pbvs placement generated: %s",result.error_msg)
        self.process_index=7
        self.process_starts[self.process_states[self.process_index]]=self.get_current_pose()
        with self._goal_handle_lock:
            
            placement_goal=PBVSPlacementGoal()
            placement_goal.desired_transform=self.load_placement_target_config(target_payload)
            placement_goal.stage1_kp=np.array([0.9]*6)
            placement_goal.stage2_kp=np.array([0.9]*6)
            placement_goal.stage3_kp=np.array([0.5]*6)
            placement_goal.stage1_tol_p=0.05
            placement_goal.stage1_tol_r=np.deg2rad(1)
            placement_goal.stage2_tol_p=0.05
            placement_goal.stage2_tol_r=np.deg2rad(1)
            placement_goal.stage3_tol_p=0.001
            placement_goal.stage3_tol_r=np.deg2rad(0.2)
            placement_goal.stage2_z_offset=0.05

            placement_goal.abort_force=Wrench(Vector3(500,500,500),Vector3(100,100,100))
            placement_goal.placement_force=Wrench(Vector3(0,0,300),Vector3(0,0,0))
            placement_goal.force_ki=np.array([1e-6]*6)
                
            client_handle=self.placement_client.send_goal(placement_goal,done_cb=done_cb)
    	
    def plan_pickup_prepare(self, target_payload, goal = None):
        
        self._begin_step(goal)
        try:
            rospy.loginfo("Begin pickup_prepare for payload %s", target_payload)
            
            object_target, object_pose=self._vision_get_object_gripper_target_pose(target_payload)
            
            self._update_payload_pose(target_payload, object_pose,parent_frame_id="pickup_nest", confidence=0.8)
            
            rospy.loginfo("Found payload %s at pose %s", target_payload, object_target)
            
            self.pose_target=copy.deepcopy(object_target)
            pose_target=self.pose_target
            pose_target.p[2] += 0.5
            
            rospy.loginfo("Prepare pickup %s at pose %s", target_payload, object_target)
            print pose_target.p
            
            path=self._plan(pose_target, config = "reposition_robot", smoother_config = "reposition_robot_smoother")

            self.current_target=target_payload
            self.state="plan_pickup_prepare"
            self.plan_dictionary['pickup_prepare']=path
            
            #rospy.loginfo("Finish pickup prepare for payload %s", target_payload)
            self._step_complete(goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)

    def move_pickup_prepare(self, mode = ControllerCommander.MODE_AUTO_TRAJECTORY, goal = None):
        self._begin_step(goal)
        try:
            self.controller_commander.set_controller_mode(mode, self.speed_scalar,[], [])
            result=None
            
            self.state="pickup_prepare"
            self.process_index=1
            self.process_starts[self.process_states[self.process_index]]=self.get_current_pose()            
            plan=self.plan_dictionary['pickup_prepare']
            self._execute_path(plan, goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)

    def plan_pickup_lower(self, goal = None):

        #TODO: check change state and target
        self._begin_step(goal)
        try:
            rospy.loginfo("Begin pickup_lower for payload %s", self.current_target)
            

            object_target, _=self._tf_get_object_gripper_target_pose(self.current_target)
            pose_target2=copy.deepcopy(object_target)
            pose_target2.p[2] += 0.3    
            print pose_target2.p

            #path=self.controller_commander.compute_cartesian_path(pose_target2, avoid_collisions=False)
            path=self._plan(pose_target2, config = "reposition_robot_short")

            self.state="plan_pickup_lower"
            self.plan_dictionary['pickup_lower']=path
            rospy.loginfo("Finish pickup_lower for payload %s", self.current_target)
            self._step_complete(goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)

    def move_pickup_lower(self, mode = ControllerCommander.MODE_AUTO_TRAJECTORY, goal = None):
        self._begin_step(goal)
        try:
            self.controller_commander.set_controller_mode(mode, 0.8*self.speed_scalar,[], self.get_payload_pickup_ft_threshold(self.current_target))
            result=None
            rospy.loginfo("moving_pickup_lower")
            if(self.state!="plan_pickup_lower"):
                self.plan_pickup_lower()
            self.state="pickup_lower"
            self.process_index=2
            self.process_starts[self.process_states[self.process_index]]=self.get_current_pose()
            path=self.plan_dictionary['pickup_lower']
            self._execute_path(path, goal)
            
        #self.execute_trajectory_action.wait_for_result()
        #self.controller_commander.execute(self.plan_dictionary['pickup_lower'])
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)

    def plan_pickup_grab_first_step(self, goal = None):
        #TODO: check change state and target
        self._begin_step(goal)
        try:
            rospy.loginfo("Begin pickup_grab for payload %s", self.current_target)
                   
            self.object_target, _=self._tf_get_object_gripper_target_pose(self.current_target)
            pose_target2=copy.deepcopy(self.object_target)
            pose_target2.p[2] -= 0.15   

            #path=self.controller_commander.compute_cartesian_path(pose_target2, avoid_collisions=False)
            path=self._plan(pose_target2, config = "panel_pickup")
            self.state="plan_pickup_grab_first_step"
            self.plan_dictionary['pickup_grab_first_step']=path
            self._step_complete(goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)

    def move_pickup_grab_first_step(self, mode = ControllerCommander.MODE_AUTO_TRAJECTORY, goal = None):
        
        self._begin_step(goal)
        
        try:
            self.controller_commander.set_controller_mode(mode, 0.8*self.speed_scalar, [],\
                                                          self.get_payload_pickup_ft_threshold(self.current_target))
            result=None
            if(self.state!="plan_pickup_grab_first_step"):
                self.plan_pickup_grab_first_step()
            self.state="pickup_grab_first_step"
            self.process_index=3
            self.process_starts[self.process_states[self.process_index]]=self.get_current_pose()
                    
            path=self.plan_dictionary['pickup_grab_first_step']
            self._execute_path(path, goal, ft_stop=True)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)

    def plan_pickup_grab_second_step(self, goal = None):
        self._begin_step(goal)
        try:
            self.rapid_node.set_digital_io("Vacuum_enable", 1)
            time.sleep(1)        
            
            #TODO: check vacuum feedback to make sure we have the panel
            
            world_to_panel_tf=self.tf_listener.lookupTransform("world", self.current_target, rospy.Time(0))
            world_to_gripper_tf=self.tf_listener.lookupTransform("world", "vacuum_gripper_tool", rospy.Time(0))
            panel_to_gripper_tf=world_to_gripper_tf.inv()*world_to_panel_tf

            #Add extra cushion for spring extension. This should be a parameter somewhere rather than hard coded.
            if(self.current_target=="leeward_mid_panel"):
                panel_to_gripper_tf.p[2]+=0.07
            else:
                panel_to_gripper_tf.p[2]+=0.05

            self.current_payload=self.current_target
            self.current_target=None
            
            self._update_payload_pose(self.current_payload, panel_to_gripper_tf, "vacuum_gripper_tool", 0.5)
            self.controller_commander.set_controller_mode(self.controller_commander.MODE_HALT,1,[],[])
            time.sleep(1)

            pose_target2=copy.deepcopy(self.object_target)
            pose_target2.p[2] += 0.20   
            

              
            #path=self.controller_commander.compute_cartesian_path(pose_target2, avoid_collisions=False)
            
            path=self._plan(pose_target2, config = "panel_pickup")

            self.state="plan_pickup_grab_second_step"
            self.plan_dictionary['pickup_grab_second_step']=path
            rospy.loginfo("Finish pickup_grab for payload %s", self.current_payload)
            self._step_complete(goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)

    def move_pickup_grab_second_step(self, mode = ControllerCommander.MODE_AUTO_TRAJECTORY, goal = None):
        self._begin_step(goal)
        try:
            self.controller_commander.set_controller_mode(mode, 0.8*self.speed_scalar,[], [])
            result=None
            if(self.state!="plan_pickup_grab_second_step"):
                self.plan_pickup_grab_second_step()
            self.state="pickup_grab_second_step"
            self.process_index=4
            self.process_starts[self.process_states[self.process_index]]=self.get_current_pose()
            
            path=self.plan_dictionary['pickup_grab_second_step']
            self._execute_path(path, goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)

    def plan_pickup_raise(self, goal = None):
        
        #TODO: check change state and target
        self._begin_step(goal)
        try:
            rospy.loginfo("Begin pickup_raise for payload %s", self.current_payload)
            
            #Just use gripper position for now, think up a better way in future
            object_target=self.tf_listener.lookupTransform("world", "vacuum_gripper_tool", rospy.Time(0))
            pose_target2=copy.deepcopy(object_target)
            pose_target2.p[2] += 0.35
            #pose_target2.p = np.array([-0.02285,-1.840,1.0])
            #pose_target2.R = rox.q2R([0.0, 0.707, 0.707, 0.0])
            
            path=self._plan(pose_target2, config = "transport_panel_short")
            
            self.state="plan_pickup_raise"
            self.plan_dictionary['pickup_raise']=path
            rospy.loginfo("Finish pickup_raise for payload %s", self.current_target)
            self._step_complete(goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)

    def move_pickup_raise(self, mode = ControllerCommander.MODE_AUTO_TRAJECTORY, goal = None):
        self._begin_step(goal)
        try:
            self.controller_commander.set_controller_mode(mode, 0.8*self.speed_scalar, [], [])
            result=None
            if(self.state!="plan_pickup_raise"):
                self.plan_pickup_raise()
            self.state="pickup_raise"
            self.process_index=5
            self.process_starts[self.process_states[self.process_index]]=self.get_current_pose()
            
            path=self.plan_dictionary['pickup_raise']
            self._execute_path(path, goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)
        
    def plan_transport_payload(self, target, goal = None):
        
        #TODO: check state and payload
        
        rospy.loginfo("Begin transport_panel for payload %s to %s", self.current_payload, target)
        self._begin_step(goal)
        try:
        
            panel_target_pose = self.tf_listener.lookupTransform("world", target, rospy.Time(0))        
            panel_gripper_pose = self.tf_listener.lookupTransform(self.current_payload, "vacuum_gripper_tool", rospy.Time(0))        
            pose_target=panel_target_pose * panel_gripper_pose            
            #pose_target.p = [1.97026484647054, 1.1179574262842452, 0.12376598588449844]
            #pose_target.R = np.array([[-0.99804142,  0.00642963,  0.06222524], [ 0.00583933,  0.99993626, -0.00966372], [-0.06228341, -0.00928144, -0.99801535]])
            #pose_target.p[1] += -0.35
            pose_target.p[2] += 0.40
    
    
            #plan=self.controller_commander.plan(pose_target)
            plan = self._plan(pose_target, config ="transport_panel", smoother_config = "transport_panel_smoother")
            
            self.current_target=target
            self.state="plan_transport_payload"
            self.plan_dictionary['transport_payload']=plan
            rospy.loginfo("Finish transport_panel for payload %s to %s", self.current_payload, target)
            self._step_complete(goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)

    def move_transport_payload(self, mode = ControllerCommander.MODE_AUTO_TRAJECTORY, goal = None):
        self._begin_step(goal)
        try:
            self.controller_commander.set_controller_mode(mode, 0.8*self.speed_scalar, [], []) 
            self.state="transport_payload"
            self.process_index=6
            self.process_starts[self.process_states[self.process_index]]=self.get_current_pose()
            plan = self.plan_dictionary['transport_payload']
            self._execute_path(plan, goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)

    def load_placement_target_config(self,target_payload):
        if(target_payload=="leeward_mid_panel"):
            transform_fname=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_process'), 'config', 'leeward_mid_panel_marker_transform.yaml')
            
        elif(target_payload=="leeward_tip_panel"):
            transform_fname=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_process'), 'config', 'leeward_tip_panel_marker_transform.yaml')

        desired_transform_msg=TransformStamped()
        
        with open(transform_fname,'r') as f:
            transform_yaml = yaml.load(f)
            
        genpy.message.fill_message_args(desired_transform_msg, transform_yaml)
        
        return desired_transform_msg
            
    def plan_gripper_release(self, goal =None):
        self._begin_step(goal)
        try:
            self.rapid_node.set_digital_io("Vacuum_enable", 0)
            #time.sleep(1)
            self.controller_commander.set_controller_mode(ControllerCommander.MODE_HALT, 0.8*self.speed_scalar,[], [])
            #TODO: check vacuum feedback to make sure we have the panel
            pose_target2=self.controller_commander.compute_fk()
            pose_target2.p[2] += 0.25
            
            #self.current_payload=self.current_target
            
            
            gripper_to_panel_tf=self.tf_listener.lookupTransform("vacuum_gripper_tool", self.current_payload, rospy.Time(0))
            world_to_gripper_tf=self.tf_listener.lookupTransform("world", "vacuum_gripper_tool", rospy.Time(0))
            world_to_panel_nest_tf=self.tf_listener.lookupTransform("world", "panel_nest", rospy.Time(0))
            panel_to_nest_tf=world_to_panel_nest_tf.inv()*world_to_gripper_tf*gripper_to_panel_tf
            
            self._update_payload_pose(self.current_payload, panel_to_nest_tf, "panel_nest", 0.5)
            
            self.current_payload=None
            self.current_target=None
            
            
            time.sleep(1)
            
            path=self._plan(pose_target2, config = "panel_pickup")

            self.state="plan_gripper_release"
            self.plan_dictionary['gripper_release']=path
            rospy.loginfo("Finished gripper release")
            self._step_complete(goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)
        
        
    def move_gripper_release(self, mode = ControllerCommander.MODE_AUTO_TRAJECTORY, goal=None):
        self._begin_step(goal)
        try:
            self.controller_commander.set_controller_mode(mode, 0.8*self.speed_scalar,[], [])
            result=None
            if(self.state!="plan_gripper_release"):
                self.plan_gripper_release()
            self.state="gripper_release"
            self.process_index=8
            self.process_starts[self.process_states[self.process_index]]=self.get_current_pose()
            
            path=self.plan_dictionary['gripper_release']
            self._execute_path(path, goal)
        except Exception as err:
            traceback.print_exc()
            self._step_failed(err, goal)
        
    def _fill_process_state(self):
        s=ProcessState()
        s.state=self.state if self.state is not None else ""
        s.payload=self.current_payload if self.current_payload is not None else ""
        s.target=self.current_target if self.current_target is not None else ""
        return s

    def publish_process_state(self):
        s=self._fill_process_state()
        self._process_state_pub.publish(s)

    def place_lower_temp(self):
        UV = np.zeros([32,2])
        P = np.zeros([32,3])
    
    def _update_payload_pose(self, payload_name, pose, parent_frame_id = None, confidence = 0.1):
        
        payload = self._get_payload(payload_name)
        
        if parent_frame_id is None:
                parent_frame_id = payload.header.frame_id
                
        parent_tf = self.tf_listener.lookupTransform(parent_frame_id, pose.parent_frame_id, rospy.Time(0))
        pose2=parent_tf.inv() * pose
        
        req=UpdatePayloadPoseRequest()
        req.name=payload_name
        req.pose=rox_msg.transform2pose_msg(pose2)
        req.header.frame_id=parent_frame_id            
        req.confidence = confidence
        
        res=self.update_payload_pose_srv(req)
        if not res.success:
            raise Exception("Could not update payload pose")
        
    def _get_payload(self, payload_name):
        payload_array_res = self.get_payload_array_srv(GetPayloadArrayRequest([payload_name]))
        if len(payload_array_res.payload_array.payloads) != 1 or payload_array_res.payload_array.payloads[0].name != payload_name:
            raise Exception("Invalid payload specified")
        
        return payload_array_res.payload_array.payloads[0]


    def _begin_step(self, goal):
        if goal is not None:
            with self._goal_handle_lock:
                if self._goal_handle is not None:
                    res = ProcessStepResult()
                    res.state=self.state
                    res.target=self.current_target if self.current_target is not None else ""
                    res.payload=self.current_payload if self.current_payload is not None else ""
                    res.error_msg="Attempt to execute new step while previous step running"
                    goal.set_rejected(result=res)                
                    rospy.loginfo("Attempt to execute new step while previous step running")
                    raise Exception("Attempt to execute new step while previous step running")
                else:
                    goal.set_accepted()
                    self._goal_handle=goal
                    
    
    def _step_complete(self, goal):
        
        if goal is not None:
            with self._goal_handle_lock:
                self.publish_process_state()
                res = ProcessStepResult()
                res.state=self.state
                res.target=self.current_target if self.current_target is not None else ""
                res.payload=self.current_payload if self.current_payload is not None else ""    
                goal.set_succeeded(res)
                
                if (self._goal_handle == goal):
                    self._goal_handle=None
    
    def _step_failed(self, err, goal):
        if goal is None:
            raise err
        else:
            with self._goal_handle_lock:
                self.publish_process_state()
                res = ProcessStepResult()
                res.state=self.state
                res.target=self.current_target if self.current_target is not None else ""
                res.payload=self.current_payload if self.current_payload is not None else ""
                res.error_msg=str(err)   
                goal.set_aborted(result=res)
                if (self._goal_handle == goal):
                    self._goal_handle=None
    
    def _execute_path(self, path, goal, ft_stop=False): 
        
        if goal is None:
            self.controller_commander.execute_trajectory(path, ft_stop=ft_stop)
            return
            
                
        def done_cb(err):
            rospy.loginfo("safe_kinematic_controller generated: %s",str(err))
            if(goal is not None):                
                if err is None:
                    self._step_complete(goal)
                else:
                    with self._goal_handle_lock:
                        if self._goal_handle == goal:
                            self._goal_handle = None
                    res = ProcessStepResult()
                    res.state=self.state
                    res.target=self.current_target if self.current_target is not None else ""
                    res.payload=self.current_payload if self.current_payload is not None else ""
                    res.error_msg=str(err)    
                    goal.set_aborted(result=res)                    
                    rospy.loginfo("safe_kinematic_controller generated: %s",str(err))
        
        with self._goal_handle_lock:
            if goal.get_goal_status().status != actionlib.GoalStatus.ACTIVE:
                if self._goal_handle == goal:
                    self._goal_handle = None
                res = ProcessStepResult()
                res.state=self.state
                res.target=self.current_target if self.current_target is not None else ""
                res.payload=self.current_payload if self.current_payload is not None else ""
                res.error_msg=str(err)
                goal.set_aborted(result = res)               
                rospy.loginfo("goal aborted before move")
                return
        
                     
            self.controller_commander.async_execute_trajectory(path, done_cb=done_cb, ft_stop=ft_stop)
        

    def _plan(self, target_pose, waypoints_pose=[], speed_scalar = 1, config = None, smoother_config = None):
       error_count=0
       while True:
           try:
               rospy.loginfo("begin rough trajectory planning with config " + str(config))
               plan1 = self.planner.trajopt_plan(target_pose, json_config_name = config)
               rospy.loginfo("rough trajectory planning complete " + str(config))
               if smoother_config is None:
                   return plan1
               rospy.loginfo("begin trajectory smoothing with config " + str(smoother_config))
               plan2=self.planner.trajopt_smooth_trajectory(plan1, json_config_name = smoother_config)
               rospy.loginfo("trajectory smoothing complete with config " + str(smoother_config))
               return plan2
           except:
               rospy.logerr("trajectory planning failed with config " + str(config) + " smoother_config " + str(smoother_config))
               error_count+=1
               if error_count > 3:                   
                   raise
               traceback.print_exc()

    
            
