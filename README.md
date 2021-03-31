# ICSL Drone - Swarm Control


This repository is for ROS package that holds the swarm control for Drones 

* Farther information for ICSL - http://icsl.hanyang.ac.kr/ 

It's compatible with 
* PX4 
[![Releases](https://img.shields.io/github/release/PX4/Firmware.svg)](https://github.com/PX4/Firmware/releases)
* Mavros [Releases 0.26](https://github.com/mavlink/mavros/releases/tag/0.26.0)
* Firmware Releases: [Downloads](https://github.com/PX4/Firmware/releases)



## Additional information
> **Note**   
> such as IP adress, port number, diractory name should be added.




Usefull alias for control :   
(*add to ~/.bashrc file*)

```bash
lias eb='gedit ~/.bashrc'
alias sb='source ~/.bashrc'
alias d1='ssh -X icsl-pi@cmila1'
alias d2='ssh -X icsl-pi@cmila2'
alias d3='ssh -X icsl-pi@cmila3'
alias gs='git status'
alias gp='git pull'
alias cw='cd ~/catkin_ws'
alias cs='cd ~/catkin_ws/src'
alias cm='cd ~/catkin_ws && catkin_make'
alias connmav='rosrun mavros mavros_node _fcu_url:=/dev/ttyUSB0:57600'
alias qgc='~/Downloads/QGroundControl.AppImage'

alias mode='rostopic pub /multi/set_mode std_msgs/String '
alias rtl='rostopic pub /multi/set_mode std_msgs/String "auto.rtl"'
alias offboard='rostopic pub /multi/set_mode std_msgs/String "offboard"'
alias takeoff='rostopic pub /multi/set_mode std_msgs/String "auto.takeoff"'
alias land='rostopic pub /multi/set_mode std_msgs/String "auto.land"'
alias arm='rostopic pub /multi/arming std_msgs/Bool 1'
alias disarm='rostopic pub /multi/set_mode std_msgs/Bool 0'
alias goto='rosservice call /multi_setpoint_local -- POINT'
alias gotov='rosservice call /swarm_node/goto_vehicle -- '
alias idle='rosservice call /swarm_node/multi_setpoint_local -- IDLE 0 0 5'
alias sp_on='rosparam set /swarm_node/setpoint/separate true'
alias sp_off='rosparam set /swarm_node/setpoint/separate false'
alias scen='rosparam set /swarm_node/scen '
alias scen4='rosservice call /swarm_node/multi_setpoint_local -- SCEN4 0 0 10'

alias tf-gpu='source activate tf-gpu'
alias tf='source activate tf'
alias deact='conda deactivate'

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/src/PX4-Autopilot/Tools/setup_gazebo.bash ~/src/PX4-Autopilot ~/src/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/PX4-Autopilot/Tools/sitl_gazebo
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export ROSLAUNCH_SSH_UNKNOWN=1
```

 <!-- mark down작성을 위한 예제들
# Heading 1
## Heading 2
### Heading 3
#### Heading 4

*This text* is italic  
**This text** is italic  
~~This text~~ is strikethrough

---


> This is a quote


[Traversy Media](http://www.traversymedia.com)

[Traversy Media](http://www.traversymedia.com "Traversy Media")

* Item 1
* Item 2
* Item 3
  * Nested Item 1
  * Nested Item 2


1. Item 1
2. Item 2
3. Item 3


![Markdown Logo](https://markdown-here.com/img/icon256.png)


```bash
  npm install

  npm start
```

```javascript
  function add(num1, num2) {
    return num1 + num2;
  }
```

```python
  def add(num1, num2):
    return num1 + num2
```

| Name     | Email          |
| -------- | -------------- |
| John Doe | john@gmail.com |
| Jane Doe | jane@gmail.com |


* [x] Task 1
* [x] Task 2
* [ ] Task 3 -->


  
