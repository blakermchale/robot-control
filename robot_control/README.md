# robot_control
ROS2 python package for controlling vehicles with common API.
## Run
Takeoff:  
```bash
ros2 action send_goal /arm_takeoff robot_control_interfaces/action/ArmTakeoff altitude:\ 5.0
```  
Land:  
```bash
ros2 action send_goal /land robot_control_interfaces/action/Land {}
```  
Go waypoint:  
```bash
ros2 action send_goal /go_waypoint robot_control_interfaces/action/GoWaypoint "{'waypoint': {'position': {'x': 10.0, 'y': 10.0, 'z': -5.0}, 'heading':0.0, 'frame': 1}}"
ros2 action send_goal /go_waypoint robot_control_interfaces/action/GoWaypoint "{'waypoint': {'position': {'x': 0.0, 'y': 0.0, 'z': -5.0}, 'heading':1.57, 'frame': 1}}"
```  
Follow waypoints:
```bash

```  
