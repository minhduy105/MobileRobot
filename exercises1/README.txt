Checking this first:

1. Source your workspace and put the file in your work space. http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
2. Test if the python in the code is executable 
3. Make sure to run those lines below to have everything update
	sudo apt-get update
	sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs

Now launch the code:
1. For the stop distance and stop type. Go to the file: "stopper.launch" to change the param. 

For the distance, change the "value" of this line
	<param name="stopDistance" value="0.5" type="double"/>

For the type of stop, change the "value" of this line from 1 of the 4["step","stepDead","prop","tanh"]
	<param name="stopType" value="stepDead"/>

For changing the K value, you can go to the file "move.py" in the source folder to change it. I set the K at 0.1. 

After that run: 
	roslaunch exercise1 stopper.launch

2. For guarded teleoperation of the turtle bot. Just run:
	roslaunch exercise1 driver.launch

Note: I hardcode step dead for this case.

3. For the video demostration, they are in the video folder.
"step", "stepDead", "prop", and "tanh" are different type of stopping (step, stepdead, proporation, tanh) for the stopping distance of 1 and 0.5
"control" is the video demonstrates teleoperate the turtle bot. 


 
Thank you for reading!
~Duy
