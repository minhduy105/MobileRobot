Hi John

You can learn how to run the code by typing: 
python planner.py -h

It will show you the syntax or you can look at my example:
python planner.py --map maze.png --radius 2 --start 60 60 --goal 100 100

The result will be in the "Result" folder:

c_space_map_with_start_and_goal.png: This is the c_space with just the start and end point
normal_map_with_start_and_goal.png: This is the normal map with just the start and end point 

goalFinding.csv: This is the result of the wavepoint(breathfirst search). Empty space is 0, wall is 1, starting point is 3

c_space_map_with_path_to_goal.png: This is the path in c_space [RESULT] 
normal_map_with_path_to_goal.png: This is the path in normal map [RESULT]

Note: as you increase the radius of the turtle bot, the start point and the path might not be close to each other. The reason is the bot is represent as a point in the center and not include its radius. The distant from the bot to when the path is the radius/body of the bot.

Note: the unit of the map is 1m^2 and the unit of the radius of the bot is 1m

Thank you for grading all of my hw and have a great summer!
~Duy 