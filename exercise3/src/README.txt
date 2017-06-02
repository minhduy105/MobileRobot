Hi John,

The code is run.py
You need to run all of the command in this link below before you run my code.
http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map

If you want it to follow the path, in my code "run.py", uncomment this line
waypoints = np.genfromtxt('point.txt', dtype=float, delimiter=',')

comment this line:
waypoints = np.genfromtxt('goal.txt', dtype=float, delimiter=',') 

If you want it to go to a point in the map, do the opposite