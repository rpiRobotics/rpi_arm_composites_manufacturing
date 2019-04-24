import os
import rospy	
import rospkg
import subprocess
import time
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from python_qt_binding.QtWidgets import QWidget, QDialog
import actionlib
from rpi_arm_composites_manufacturing_process.msg import ProcessStepAction, ProcessStepGoal, ProcessState
import threading
from safe_kinematic_controller.msg import ControllerMode


class GUI_Step_Executor(QObject):
    
    error_signal= pyqtSignal()
    success_signal=pyqtSignal()
    
    def __init__(self):
        super(GUI_Step_Executor, self).__init__()
        self.in_process=None
        self.recover_from_pause=False
        self.rewound=False

        self.execute_states=[['plan_reset_position','move_reset_position'],['plan_pickup_prepare','move_pickup_prepare'],['plan_pickup_lower','move_pickup_lower','plan_pickup_grab_first_step','move_pickup_grab_first_step','plan_pickup_grab_second_step','move_pickup_grab_second_step','plan_pickup_raise','move_pickup_raise'],
                            ['plan_transport_payload','move_transport_payload'],['place_panel','plan_gripper_release','move_gripper_release'],['plan_place_set_second_step'],['stop_motion'],['plan_rewind_motion','move_rewind_motion']]
        
        self.client=actionlib.SimpleActionClient('process_step', ProcessStepAction)
        self.client.wait_for_server()
        self.client_handle=None
        self.start_step=0
        self.commands_sent=False
        self.current_state=0
        self.current_command=0
        self.target_index=-1
        self.target=None
        self.error=None
        self.state=None
        self.gui_state_pub = rospy.Publisher("GUI_state", ProcessState, queue_size=100, latch=True)
        self.controller_mode = ControllerMode.MODE_AUTO_TRAJECTORY
        #rospy.Subscriber("process_state",ProcessState,self._next_command)
        
        
    #Receives error messages from process controller action calls and signals gui to put up alert
    def _feedback_receive(self,status,result):
        """
        emits signal to qt thread when error is received from process controller
        
        """
        rospy.loginfo("Feedback_receive_function")
        self.recover_from_pause=True
        self.error=result.error_msg
        self.error_signal.emit()
        
        self.success_signal.emit()
    
    #callback triggered by process_state message subscription that makes the next process action call 
    #def _next_command(self,data):
    def _next_command(self,status,result):
        """
        the process controller publishes to process_state when commands are completed successfully
        data is the published process state data it is not actually checked, the GUI instead manages its own state machine 
        self.execute_states is a list of all the process commands in the process_controller
        self.recover_from_pause is used as a flag to stop sending the rest of the commands in the sequence if pause is commanded
        self.current_state is the first index for self.execute_states and is only changed by the user pressing a button
        self.current_command is the second index for self.execute_states and is incremented in this function, this function is called until self.current_command>=len(self.execute_states) at which point the GUI step has been completed
        self.target_index is used to signal which step should contain the target being interacted with
        self.target is the target id of the operation
        ProcessStepGoal is the action goal sent to the process_controller action server
        self.client_handle is the variable used to store the current action_client_handle in case a pause is commanded
        self._feedback_receive is the error received callback function for the action client
        self._publish_state_message is a function called to alert the main GUI program that the GUI step has been completed, this is used to reenable the GUI command buttons
        """
        if(status==actionlib.GoalStatus.SUCCEEDED):
            
    	    rospy.loginfo("Next_command")
            if(self.recover_from_pause):
                return
            self.state=self.execute_states[self.current_state][self.current_command]
            self.current_command+=1
            
            rospy.loginfo("current state %i"%self.current_state)
            rospy.loginfo("current command %i"%self.current_command)
            if(not(self.current_command>=len(self.execute_states[self.current_state]))):
                
                
                if(self.current_command==self.target_index):
                    g=ProcessStepGoal(self.execute_states[self.current_state][self.current_command], self.target, ControllerMode(self.controller_mode))
                else:
                    g=ProcessStepGoal(self.execute_states[self.current_state][self.current_command], "", ControllerMode(self.controller_mode))
                #self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive)
                #self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive,done_cb=self._next_command)
                self.client_handle=self.client.send_goal(g,done_cb=self._next_command)
                #self.client.wait_for_result()
            else:
                self.success_signal.emit()
        else:
            rospy.loginfo("Feedback_receive")
            self.error=result.error_msg
            self.error_signal.emit()
        
            self.success_signal.emit()

    def _execute_steps(self,steps_index, target="",target_index=-1):
        """
        function called to send the first command to the process controller, it takes as arguments the following
        steps_index is the index of the steps to execute according to the list self.execute_states
        target is the panel or nest target to be sent to the process controller
        target_index is the index of the command in which the target is to be sent as a parameter to the process controller
        """

        
        def send_action(goal):
            """a simple closure to send the action goal"""
            #self.client_handle=self.client.send_goal(goal,feedback_cb=self._feedback_receive)
            #self.client_handle=self.client.send_goal(goal,feedback_cb=self._feedback_receive,done_cb=self._next_command)
            self.client_handle=self.client.send_goal(goal,done_cb=self._next_command)
            #self.client.wait_for_result()
        
        """because this function is also called if a motion is to be resumed after stopping a start_step value is used, normally zero, if recover_from_pause is true then it will resume the process
           starting at the last planning step


        """
        self.start_step=0
        self.current_state=steps_index
        self.target_index=target_index
        self.target=target
        #for step_num in range(resume_index,len(self.execute_states[steps_index])):
        if(self.recover_from_pause):
            if(steps_index!=0 and steps_index!=7):
                if('plan' in self.execute_states[steps_index][self.current_command]):
                    self.start_step=self.current_command
                else:
                    self.start_step=self.current_command-1
            else:
                pass

            self.recover_from_pause=False

        self.current_command=self.start_step
        self.state=self.execute_states[self.current_state][self.current_command]
        if(self.start_step==target_index):
            g=ProcessStepGoal(self.execute_states[steps_index][self.start_step], target, ControllerMode(self.controller_mode))
        else:
            g=ProcessStepGoal(self.execute_states[steps_index][self.start_step], "", ControllerMode(self.controller_mode))
            	
        	
		
            #self._send_event.wait()
        
        
		
        send_action(g)
            #self.in_process=True

        



        self.commands_sent=True

        #if( not self.recover_from_pause):
        #self.last_step=0

        
      
    def _nextPlan(self,panel_type,planListIndex,panel_nest=None):
        """
        the function initially called by the experiment_gui code


        """
        rospy.loginfo("next plan planListIndex: "+str(planListIndex))

        #if self.recover_from_pause and planListIndex !=0:
        #    planListIndex-=1

        #self._open_rviz_prompt()
        #self._raise_rviz_window()



        time.sleep(1)

        if(planListIndex==0):
            
            self._execute_steps(0)
            '''try:
                g=ProcessStepGoal(self.execute_states[0][0], "")
                self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive)
                #self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive,done_cb=self._next_command)
            finally:
                pass
                #self._publish_state_message()
            '''
            

        elif(planListIndex==1):
            
            self._execute_steps(1,panel_type,0)
            
            

        elif(planListIndex==2):
            
            self._execute_steps(2)
            
        

        elif(planListIndex==3):
            
            self._execute_steps(3,panel_nest,0)

        elif(planListIndex==4):
            
            
            self._execute_steps(4,panel_type,0)

            

        if(self.rewound):
            self.rewound=False
            


    def _previousPlan(self):
        try:
            self._execute_steps(7)
            #g=ProcessStepGoal(self.execute_states[7][0], "", ControllerMode(self.controller_mode))
            #self.client_handle=self.client.send_goal(g,done_cb=self._next_command)
            #self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive,done_cb=self._next_command)
        finally:
            pass
        self.rewound=True


    def _stopPlan(self):
        client=self.client
        #
        #self.controller_commander.set_controller_mode(self.controller_commander.MODE_HALT, 0,[], [])
        #client.cancel_all_goals()
        rospy.loginfo("cancelled all goals")
        #g=ProcessStepGoal(self.execute_states[6][0], "")
        #self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive)
        client.cancel_all_goals()
        
        self.recover_from_pause=True

    def _publish_state_message(self):
        s=ProcessState()
        s.state=str(self.execute_states[self.current_state])
        s.payload=""
        s.target=""
        self.gui_state_pub.publish(s)



