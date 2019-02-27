# Demo 4

## Running Physical Navigation
Order is important: \[optional]
- `roslaunch demo4 waypoint_config.launch`
- `roslaunch demo4 nav_stack.launch [map_file]`
- `rviz` -> Open config: NaigationStackRviz.rviz

### If errors occur:
Restart everything (including Turtlebot), and run the above commands in order

### To navigate the robot:
While the other nodes are running, created a 2D Goal location in RVIZ. The robot should navigate to this location
 in the map