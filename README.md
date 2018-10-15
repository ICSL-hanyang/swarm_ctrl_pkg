Drone Setpoint Test code

plz insert the code below into .bashrc 

alias d0='ssh -X icsl-pi@camila0'
alias d1='ssh -X icsl-pi@camila1'
alias d2='ssh -X icsl-pi@camila2'
alias d3='ssh -X icsl-pi@camila3'

alias mode='rostopic pub /multi/set_mode std_msgs/String '
alias offboard='rostopic pub /multi/set_mode std_msgs/String "offboard"'
alias arm="rostopic pub /multi/arming std_msgs/Bool 1"
alias disarm="rostopic pub /multi/arming std_msgs/Bool 0"
alias takeoff="rostopic pub /multi/set_mode std_msgs/String 'auto.takeoff'"
alias land="rostopic pub /multi/set_mode std_msgs/String 'auto.land'"
alias goto='rosservice call /multi_setpoint_local -- '