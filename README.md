Drone Setpoint Test code

plz insert the code below into .bashrc 

alias offboard="rosservice call /multi_mode 'OFFBOARD'"
alias arm='rosservice call /multi_arming 1'
alias takeoff="rosservice call /multi_mode 'AUTO.TAKEOFF'"
alias land="rosservice call /multi_mode 'AUTO.LAND'"
alias goto='rosservice call /multi_set_pos_local -- 1 '
