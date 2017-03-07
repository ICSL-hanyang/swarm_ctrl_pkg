Drone Setpoint Test code

plz insert the code below into .bashrc 

alias take='rosparam set /test_node/mode 0'
alias land='rosparam set /test_node/mode 4'
alias x='rosparam set /test_node/x'
alias y='rosparam set /test_node/y'
alias z='rosparam set /test_node/z'
