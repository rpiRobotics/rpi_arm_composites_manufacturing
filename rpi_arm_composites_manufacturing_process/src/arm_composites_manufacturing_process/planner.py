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

import rospy
from tesseract_msgs.msg import TesseractState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tesseract
import threading
import rospkg
import os
import numpy as np
import general_robotics_toolbox as rox
import general_robotics_toolbox.ros_msg as rox_msg
from geometry_msgs.msg import Pose, PoseArray


class Planner(object):
    
    def __init__(self, controller_commander, urdf_xml_string, srdf_xml_string=None):
        
        self.tesseract_env=tesseract.KDLEnv()
        self.tesseract_env.init(urdf_xml_string, srdf_xml_string)
        self.tesseract_diff_sub=rospy.Subscriber("tesseract_diff", TesseractState, self._tesseract_diff_cb)
        self.tesseract_plotter = tesseract.ROSBasicPlotting(self.tesseract_env)
        self.tesseract_plotter.plotScene()
        self.controller_commander=controller_commander
        self.waypoint_plotter = rospy.Publisher("process_planner_waypoints", PoseArray, queue_size=5)
        self.lock=threading.Lock()  
    
    def _tesseract_diff_cb(self, msg):
        with self.lock:
            tesseract.processTesseractStateMsg(self.tesseract_env, msg)
            self.tesseract_plotter.plotScene()
    
    def load_json_config(self, config_name, package_name = 'rpi_arm_composites_manufacturing_process'):
        r = rospkg.RosPack()
        package_path = r.get_path(package_name)
        file_path=os.path.join(package_path, 'config', config_name + '.json')
        with open(file_path, 'r') as f:
            return f.read()

    def trajopt_plan(self, target_pose, json_config_str=None, json_config_name=None, target_joints=None):
        
        with self.lock:
        
            if (json_config_str is None and json_config_name is not None):
                json_config_str = self.load_json_config(json_config_name)
        
            robot = self.controller_commander.rox_robot
            
            #vel_upper_lim = np.array(robot.joint_vel_limit) * speed_scalar
            #vel_lower_lim = -vel_upper_lim
            #joint_lower_limit = np.array(robot.joint_lower_limit)
            #joint_upper_limit = np.array(robot.joint_upper_limit)
            joint_names = robot.joint_names            
            
            joint_positions = self.controller_commander.get_current_joint_values()
            
            if target_pose is not None:
                p = PoseArray()
                p.header.frame_id="world"
                p.header.stamp=rospy.Time.now()
                p.poses.append(rox_msg.transform2pose_msg(self.controller_commander.compute_fk(joint_positions)))
                p.poses.append(rox_msg.transform2pose_msg(target_pose))
                self.waypoint_plotter.publish(p)            
            
            self.tesseract_env.setState(joint_names, joint_positions)
            
            init_pos = self.tesseract_env.getCurrentJointValues()
            self.tesseract_plotter.plotTrajectory(self.tesseract_env.getJointNames(), np.reshape(init_pos,(1,6)));
        
            planner = tesseract.TrajOptPlanner()
            
            manip="move_group"
            end_effector="vacuum_gripper_tool"
            
            pci = tesseract.ProblemConstructionInfo(self.tesseract_env)
    
            pci.fromJson(json_config_str)
    
            pci.kin = self.tesseract_env.getManipulator(manip)
                        
            pci.init_info.type = tesseract.InitInfo.STATIONARY
            #pci.init_info.dt=0.5
            
            if target_pose is not None:
                #Final target_pose constraint
                pose_constraint = tesseract.CartPoseTermInfo()
                pose_constraint.term_type = tesseract.TT_CNT
                pose_constraint.link = end_effector
                pose_constraint.timestep = pci.basic_info.n_steps-1            
                q=rox.R2q(target_pose.R)            
                pose_constraint.wxyz = np.array(q)
                pose_constraint.xyz = np.array(target_pose.p)
                pose_constraint.pos_coefs = np.array([1000000,1000000,1000000], dtype=np.float64)
                pose_constraint.rot_coefs = np.array([10000,10000,10000], dtype=np.float64)
                pose_constraint.name = "final_pose"
                pci.cnt_infos.push_back(pose_constraint)
            elif target_joints is not None:
                joint_constraint = tesseract.JointPosTermInfo()
                joint_constraint.term_type = tesseract.TT_CNT
                joint_constraint.link = end_effector
                joint_constraint.first_step = pci.basic_info.n_steps-2
                joint_constraint.last_step = pci.basic_info.n_steps-1
                #joint_constraint.coeffs = tesseract.DblVec([10000]*6)
                joint_constraint.targets = tesseract.DblVec(list(target_joints))
                print target_joints
                pci.cnt_infos.push_back(joint_constraint)
            else:
                assert False
        
                            
            prob = tesseract.ConstructProblem(pci)
            
            config = tesseract.TrajOptPlannerConfig(prob)
            
            config.params.max_iter = 1000
            
            planning_response = planner.solve(config)
            
            if (planning_response.status_code != 0):
                raise Exception("TrajOpt trajectory planning failed with code: %d" % planning_response.status_code )
            
            self.tesseract_plotter.plotTrajectory(self.tesseract_env.getJointNames(), planning_response.trajectory[:,0:6])
            
            jt = JointTrajectory()
            jt.header.stamp = rospy.Time.now()
            jt.joint_names = joint_names
            
            trajectory_time = np.cumsum(1.0/planning_response.trajectory[:,6])
            trajectory_time = trajectory_time - trajectory_time[0]
            
            for i in xrange(planning_response.trajectory.shape[0]):
                jt_p=JointTrajectoryPoint()
                jt_p.time_from_start=rospy.Duration(trajectory_time[i])
                jt_p.positions = planning_response.trajectory[i,0:6]
                jt.points.append(jt_p)
            
            return jt
    
    def trajopt_smooth_trajectory(self, trajectory_in, json_config_str=None, json_config_name=None):
        
        with self.lock:
        
            if (json_config_str is None and json_config_name is not None):
                json_config_str = self.load_json_config(json_config_name)
        
            robot = self.controller_commander.rox_robot
            
            #vel_upper_lim = np.array(robot.joint_vel_limit) * speed_scalar
            #vel_lower_lim = -vel_upper_lim
            #joint_lower_limit = np.array(robot.joint_lower_limit)
            #joint_upper_limit = np.array(robot.joint_upper_limit)
            joint_names = trajectory_in.joint_names
            
            joint_positions = trajectory_in.points[0].positions
            
            self.tesseract_env.setState(joint_names, joint_positions)
            
            init_pos = self.tesseract_env.getCurrentJointValues()
            self.tesseract_plotter.plotTrajectory(self.tesseract_env.getJointNames(), np.reshape(init_pos,(1,6)));
        
            planner = tesseract.TrajOptPlanner()
            
            manip="move_group"
            end_effector="vacuum_gripper_tool"
            
            pci = tesseract.ProblemConstructionInfo(self.tesseract_env)
    
            pci.fromJson(json_config_str)
    
            pci.kin = self.tesseract_env.getManipulator(manip)
                        
            pci.init_info.type = tesseract.InitInfo.STATIONARY
            #pci.init_info.dt=0.5
            
            for i in xrange(14):
                joint_constraint = tesseract.JointPosTermInfo()
                joint_constraint.targets = tesseract.DblVec(trajectory_in.points[i].positions)
                joint_constraint.first_step = i*10
                joint_constraint.last_step = i*10
                joint_constraint.upper_tols = tesseract.DblVec(np.array([np.deg2rad(0.1)]*6))
                joint_constraint.lower_tols = tesseract.DblVec(np.array([np.deg2rad(-0.1)]*6))
                joint_constraint.term_type = tesseract.TT_COST
                joint_constraint.coeffs = tesseract.DblVec([1000]*6)
                pci.cost_infos.push_back(joint_constraint)
            
            joint_constraint = tesseract.JointPosTermInfo()
            print trajectory_in.points[-1].positions
            joint_constraint.targets = tesseract.DblVec(trajectory_in.points[-1].positions)
            joint_constraint.first_step = pci.basic_info.n_steps-1
            joint_constraint.last_step = pci.basic_info.n_steps-1
            #joint_constraint.upper_tols = tesseract.DblVec(np.array([np.deg2rad(0.1)]*6))
            #joint_constraint.lower_tols = tesseract.DblVec(np.array([np.deg2rad(-0.1)]*6))
            joint_constraint.coeffs = tesseract.DblVec([100000]*6)
            joint_constraint.term_type = tesseract.TT_COST
            pci.cost_infos.push_back(joint_constraint)
            
            prob = tesseract.ConstructProblem(pci)
            
            config = tesseract.TrajOptPlannerConfig(prob)
            
            config.params.max_iter = 1000
            
            planning_response = planner.solve(config)
            
            if (planning_response.status_code != 0):
                raise Exception("TrajOpt trajectory planning failed with code: %d" % planning_response.status_code )
            
            self.tesseract_plotter.plotScene()
            self.tesseract_plotter.plotTrajectory(self.tesseract_env.getJointNames(), planning_response.trajectory[:,0:6])
            
            jt = JointTrajectory()
            jt.header.stamp = rospy.Time.now()
            jt.joint_names = joint_names
            
            trajectory_time = np.cumsum(1.0/planning_response.trajectory[:,6])
            trajectory_time = trajectory_time - trajectory_time[0]
            
            for i in xrange(planning_response.trajectory.shape[0]):
                jt_p=JointTrajectoryPoint()
                jt_p.time_from_start=rospy.Duration(trajectory_time[i])
                jt_p.positions = planning_response.trajectory[i,0:6]
                jt.points.append(jt_p)
            
            return jt
    
        