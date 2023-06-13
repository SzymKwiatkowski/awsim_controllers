# AWSIM controllers

Repository contains 4 controllers:
- [x] Pure pursuit controller
- [x] Stanley controller
- [x] ILQR controller
- [ ] ICEM controller

## Requirements
First thing required is having f1tenth simulator running along with ros humble. Second requirement is to create new ros workspace and clone this repository to src directory as well as autoware_auto_msgs with commands:
```bash
git clone https://github.com/SzymKwiatkowski/awsim_controllers
```
and
```bash
git clone https://github.com/tier4/autoware_auto_msgs
```
After that build autoware messages with command:
```bash
colcon build --packages-select autoware_auto_control_msgs
```
After wards source to with:
```bash
source install/setup.bash
```
And then build entire workspace with:
```bash
colcon build --symlink-install
```

# Running controllers with simulators
There is an option of running any controller along with simulator right of the bat. The only thing needed is to have simulator installed. When it is installed and running in background using controllers is conducted with simple ros commands.

## Pure pursuit controller
Running pure pursuit is conducted via following command:
```bash
ros2 launch awsim_controllers pp_controller_launch.py 
```

## Stanley controller
Running stanley is conducted via following command:
```bash
ros2 launch awsim_controllers stanley_controller_launch.py
```
**WARNING!** Using this controller takes very long to actually initialize

## ILQR controller
Running [iLQR](https://github.com/macnack/iLQR) is conducted via following command:
```bash
ros2 launch awsim_controllers ilqr_controller_launch.py
```
**WARNING!** Using this controller takes very very long to actually initialize