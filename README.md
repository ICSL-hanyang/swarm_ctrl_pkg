Drone Setpoint Test code

plz insert the code below into .bashrc 

alias offboard="rosservice call /multi_mode 'OFFBOARD'"
alias arm='rosservice call /multi_arming 1'
alias takeoff='rosservice call /multi_set_pos_local 1 0 0 3'
alias goto='rosservice call /multi_set_pos_local 1 '
alias land="rosservice call /multi_landing 'here'"
alias reset="rosservice call /multi_set_vel_local 0 0 0 0"
