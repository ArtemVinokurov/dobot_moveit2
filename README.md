# DOBOT_MOVEIT2
Moveit 2 configuration package for Dobot CR16 

## Usage
### Rviz only
To run visualization in Rviz without a real robot, use the command:
```
ros2 launch dobot_moveit_config dobot_moveit.launch.py use_fake_hardware:=true
```
### Real robot
To run the package on a real manipulator, specify the current ip address of the robot controller in the robot_ip argument:
```
ros2 launch dobot_moveit_config dobot_moveit.launch.py robot_ip:=X.X.X.X
```
