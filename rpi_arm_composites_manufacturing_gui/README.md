RPI ARM Composites manufacturing GUI

This ROS package is an RQT plugin designed to integrate with the rpi_arm_composites_manufacturing_process and safe_kinematic_controller 
The RQT gui is designed to allow the user to step through a list of overall process steps which may consist of a number of action commands

 
To use this GUI install the overall system then run the overall manufacturing run.launch file to launch rqt or launch rqt from another command line, then select Experiment GUI 
from the plugin dropdown menu.

The GUI relies on the process controller's action server and will not open if the process controller action server has not successfully started

Operating the GUI
The GUI consists of two screens, a welcome screen and a runscreen. The welcome screen has several status LEDs which check the status of
the robot connection, the force torque sensor, the overhead camera and the gripper camera. It updates these based on a subscription to controller_state ROS msgs published by the safe_kinematic_controller.
The overhead camera and gripper camera are simply triggers checked by the GUI to be in the available service list. To proceed to the runscreen the user must press the Next Screen button. 
This button opens up a menu to allow the user to select the panel type that they wish to pick and place.

The runscreen consists of a Welcome screen button, a plan list, a robot status indicator, a teleop-mode/error recovery button, a shared control button, several operations buttons, a panel type
and placement location display, a skip command button and a force-torque data display.
The Welcome screen button returns the user to the Welcome screen, the plan list details all the GUI process steps, each one contains multiple action calls, during operations and movement the
step that the robot is moving to and the previous step will be highlighted gray with red font, upon completion of the step the font will turn green for that step. If an error occurs then
the step will highlight yellow and the font will turn red. The robot status indicator is green when the robot is connected and not in an error mode according to the safe_kinematic_controller.
The teleop button is used normally to transition between the different available teleop modes which include: Off, Joint, Cartesian, Cylindrical, and Spherical. Pressing the button iterates through
and cycles back to Off. If the safe_kinematic_controller enters a negative controller mode then this button becomes an error recovery button and displays the error code of the controller, pressing
the button in this mode will attempt to set the controller back into mode 0 (Halt). THIS MUST BE DONE BEFORE MOVING OR THE ROBOT WILL NOT BE SYNCHED AND THE CONTROLLER WILL ERROR. The shared control
button enables shared control of movements in the controller, the button will turn orange when selected, then the user must step through commands normally. Click the button again to set motions back
in AUTO mode. 
The command buttons available to the system are Rewind, Home, Pause and Next. Rewind sends a command to plan and execute motion back to the starting posiion of the last executed command. This does not work
for all commands, you cannot rewind movements to home and some command steps involving the pickup and placement steps. In addition if you pause in the execution of a motion, then proceed with the motion
again the rewind will go to the point at which the motion was restarted, in which case YOU SHOULD NOT PRESS THE REWIND BUTTON AGAIN or system behavior could be unpredictable.
The Home button opens a confirmation window to ensure user approval since planning from any location could lead to unpredictable behavior. It is recommended that this motion not be used while the panel
is held by the robot as planning will be more difficult. The pause button is disabled normally but during motion can be used to stop the motion THIS IS SLOW AND SHOULD NOT REPLACE E-STOP. 
Next step is the primary command to iterate through the sequence, as with all motions the button is disabled after being commanded to prevent multiple clicks and reenabled on error or completion of step.
In the case of an error occuring the Next step button becomes a Retry button and will allow the user to retry execution of the previous command.
The panel type and placement location displays show which panel is currently being selected and sent in commands to the process_controller. This should
be checked before proceeding with operation, if incorrect panel type is selected then the first step in sequence will error with Requested Object Not Found.
The skip command button SHOULD NOT BE USED. It is present in case of system failure to allow the user to skip to the failed step and try to finish the placement. SKIPPING CERTAIN STEPS WILL CAUSE ERRORS.
The force-torque display is used to display the current readings of the force torque sensor received from the safe_kinematic_controller's controller_state message. These readings are downsampled by a factor of 10
to prevent overloading the update of the display. 
