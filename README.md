# AWSIM controllers

Repository contains 4 controllers:
- [x] Pure pursuit controller
- [x] Stanley controller
- [ ] ILQR controller
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
