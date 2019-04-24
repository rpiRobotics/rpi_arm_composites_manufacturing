from execute_gui_steps import GUI_Step_Executor
from rpi_arm_composites_manufacturing_gui.msg import GUIStepAction, GUIStepGoal, GUIStepResult
import rospy
import actionlib
class GUIExecutionServer(object):
    def __init__(self):
        self.executor=GUI_Step_Executor()
        self.server=actionlib.SimpleActionServer("gui_step", GUIStepAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()


    def execute_cb(self, goal):

        command = goal.command
        if command == "reset":
            self.executor._nextPlan(goal.target,0)
        elif command == "panel_pickup":
            self.executor._nextPlan(goal.target,1)
        elif command == "pickup_grab":
            self.executor._nextPlan(goal.target,2)
        elif command == "transport_panel":
            self.executor._nextPlan(goal.target,3)
        elif command == "place_panel":
            self.executor._nextPlan(goal.target,4)
        elif command == "previous_plan":
            self.executor._previousPlan()
        elif command == "stop_plan":
            self.executor._stopPlan()
        else:
            assert False, "Invalid command"

        res = GUIStepResult()
        res.state=goal.command
        res.target=goal.target
        

        self.server.set_succeeded(res)

def gui_action_server_main():
    rospy.init_node("gui_action_server")

    s=GUIExecutionServer()
    

    rospy.spin()
