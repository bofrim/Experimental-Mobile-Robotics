# Demo 4

## Running Physical Navigation
Order is important:
`roslaunch demo4 waypoint_config.launch`
`roslaunch demo4 nav_stack.launch`
`rviz` -> Open config: NaigationStackRviz.rviz

###If errors occur:
Restart everything (including Turtlebot), and run the above commands in order

###To navigate the robot:
In RVIZ press the 2D Goal on a part of the map. The robot should navigate to that location
