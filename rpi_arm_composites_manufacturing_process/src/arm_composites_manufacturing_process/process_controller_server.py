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

from __future__ import absolute_import
from .arm_composites_manufacturing_process import ProcessController
from rpi_arm_composites_manufacturing_process.msg import ProcessStepAction, ProcessStepResult
import actionlib
import rospy

class ProcessControllerServer(object):
    def __init__(self, disable_ft=False):
        self.controller=ProcessController(disable_ft)
        self.server=actionlib.ActionServer("process_step", ProcessStepAction, goal_cb=self.execute_cb,cancel_cb=self.cancel, auto_start=False)
        #TODO make goal cancelling of goal handle start here
        self.server.start()
                
    def cancel(self,goal):
        self.controller.cancel_step(goal)
        
    def execute_cb(self, goal):        
        command = goal.get_goal().command
        target = goal.get_goal().target
        mode = goal.get_goal().mode.mode
        
        if command == "plan_pickup_prepare":
            self.controller.plan_pickup_prepare(target, goal)
        elif command == "stop_motion":
            self.controller.stop_motion(goal)
        elif command == "move_pickup_prepare":
            self.controller.move_pickup_prepare(mode, goal)
        elif command == "plan_to_reset_position":
            self.controller.plan_to_reset_position(goal)
        elif command == "plan_pickup_lower":
            self.controller.plan_pickup_lower(goal)
        elif command == "move_pickup_lower":
            self.controller.move_pickup_lower(mode, goal)
        elif command == "plan_pickup_grab_first_step":
            self.controller.plan_pickup_grab_first_step(goal)
        elif command == "move_pickup_grab_first_step":
            self.controller.move_pickup_grab_first_step(mode, goal)
        elif command == "plan_pickup_grab_second_step":
            self.controller.plan_pickup_grab_second_step(goal)
        elif command == "move_pickup_grab_second_step":
            self.controller.move_pickup_grab_second_step(mode, goal)
        elif command == "plan_pickup_raise":
            self.controller.plan_pickup_raise(goal)
        elif command == "move_pickup_raise":
            self.controller.move_pickup_raise(mode, goal)
        elif command == "plan_transport_payload":
            self.controller.plan_transport_payload(target, goal)
        elif command == "move_transport_payload":
            self.controller.move_transport_payload(mode, goal)
        elif command == "plan_place_lower":
            self.controller.plan_place_lower()
        elif command == "move_place_lower":
            self.controller.move_place_lower()
        elif command == "plan_place_set_first_step":
            self.controller.plan_place_set_first_step()
        elif command == "move_place_set_first_step":
            self.controller.move_place_set_first_step()
        elif command == "plan_place_set_second_step":
            self.controller.plan_place_set_second_step()
        elif command == "move_place_set_second_step":
            self.controller.move_place_set_second_step()
        elif command == "plan_place_raise":
            self.controller.plan_place_raise()
        elif command == "move_place_raise":
            self.controller.move_place_raise()
        elif command == "plan_reset_position":
            self.controller.plan_reset_position(goal)
        elif command == "move_reset_position":
            self.controller.move_reset_position(mode, goal)
        elif command == "plan_gripper_release":
            self.controller.plan_gripper_release(goal)
        elif command == "move_gripper_release":
            self.controller.move_gripper_release(mode, goal)
        elif command == "place_panel":
            self.controller.place_panel(target,goal)
        elif command == "plan_rewind_motion":
            self.controller.plan_rewind_motion(goal)
        elif command == "move_rewind_motion":
            self.controller.move_rewind_motion(mode,goal)
        else:
            goal.set_rejected()
            assert False, "Invalid command"
            
        #rospy.loginfo(goal.get_goal_status().status)
               
            
def process_controller_server_main():
    rospy.init_node("process_controller_server")
    
    disable_ft=rospy.get_param('~disable_ft', False)
    
    s=ProcessControllerServer(disable_ft=disable_ft)
    
    rospy.spin()
