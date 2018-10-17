Running PX4 in SITL Mode
==========================

Steps
---------------------------

1. Download PX4 v1.8.0 Firmware
```
git clone https://github.com/PX4/Firmware.git --branch v1.8.0  
cd Firmware
git submodule update --init --recursive
make posix_sitl_default
make posix_sitl_default sitl_gazebo
```

2. Source your environment:
```bash
source ~/Drone/Firmware/Tools/setup_gazebo.bash ~/Drone/Firmware ~/Drone/Firmware/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Drone/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Drone/Firmware/Tools/sitl_gazebo
```
3. launch
```
roslaunch swarm_ctrl_pkg multi_uav_mavros_sitl.launch 
```
Detailed settings for multi_UAV gazebo
---------------------------
  
There is a sample startup script at `swarm_ctrl_pkg/config/posix-configs/SITL/init`
 which looks like 
 ```
 uorb start
param load
dataman start
param set MAV_SYS_ID 1     <<<<<<<<<<<<<
param set BAT_N_CELLS 3
param set CAL_ACC0_ID 1376264
param set CAL_ACC0_XOFF 0.01
param set CAL_ACC0_XSCALE 1.01
param set CAL_ACC0_YOFF -0.01
param set CAL_ACC0_YSCALE 1.01
param set CAL_ACC0_ZOFF 0.01
param set CAL_ACC0_ZSCALE 1.01
param set CAL_ACC1_ID 1310728
param set CAL_ACC1_XOFF 0.01
param set CAL_GYRO0_ID 2293768
param set CAL_GYRO0_XOFF 0.01
param set CAL_MAG0_ID 196616
param set CAL_MAG0_XOFF 0.01
param set COM_DISARM_LAND 3
param set COM_OBL_ACT 2
param set COM_OBL_RC_ACT 0
param set COM_OF_LOSS_T 5
param set COM_RC_IN_MODE 1
param set EKF2_AID_MASK 1
param set EKF2_ANGERR_INIT 0.01
param set EKF2_GBIAS_INIT 0.01
param set EKF2_HGT_MODE 0
param set EKF2_MAG_TYPE 1
param set MAV_TYPE 2
param set MC_PITCH_P 6
param set MC_PITCHRATE_P 0.2
param set MC_ROLL_P 6
param set MC_ROLLRATE_P 0.2
param set MIS_TAKEOFF_ALT 2.5
param set MPC_HOLD_MAX_Z 2.0
param set MPC_Z_VEL_I 0.15
param set MPC_Z_VEL_P 0.6
param set NAV_ACC_RAD 2.0
param set NAV_DLL_ACT 2
param set RTL_DESCEND_ALT 5.0
param set RTL_LAND_DELAY 5
param set RTL_RETURN_ALT 30.0
param set SDLOG_DIRS_MAX 7
param set SENS_BOARD_ROT 0
param set SENS_BOARD_X_OFF 0.000001
param set SITL_UDP_PRT 14560     <<<<<<<<<<<<<
param set SYS_AUTOSTART 4010
param set SYS_MC_EST_GROUP 2
param set SYS_RESTART_TYPE 2
replay tryapplyparams
simulator start -s 
tone_alarm start
gyrosim start
accelsim start
barosim start
gpssim start
pwm_out_sim start
sensors start
commander start
land_detector start multicopter
navigator start
ekf2 start
mc_pos_control start
mc_att_control start
mixer load /dev/pwm_output0 ROMFS/px4fmu_common/mixers/quad_w.main.mix
mavlink start -x -u 14556 -r 4000000                        
mavlink start -x -u 14557 -r 4000000 -m onboard -o 14540    <<<<<<<<<<<<<
mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u 14556
mavlink stream -r 50 -s LOCAL_POSITION_NED -u 14556
mavlink stream -r 50 -s GLOBAL_POSITION_INT -u 14556
mavlink stream -r 50 -s ATTITUDE -u 14556
mavlink stream -r 50 -s ATTITUDE_QUATERNION -u 14556
mavlink stream -r 50 -s ATTITUDE_TARGET -u 14556
mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u 14556
mavlink stream -r 20 -s RC_CHANNELS -u 14556
mavlink stream -r 250 -s HIGHRES_IMU -u 14556
mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u 14556
logger start -e -t
mavlink boot_complete
replay trystart

 ```
 and `<<<<<<<<<<<<<` line is where you have to modify  
 ## for QGC port number `14556` should be modified
 ```
 mavlink start -x -u 14556 -r 4000000 
 mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u 14556
mavlink stream -r 50 -s LOCAL_POSITION_NED -u 14556
mavlink stream -r 50 -s GLOBAL_POSITION_INT -u 14556
mavlink stream -r 50 -s ATTITUDE -u 14556
mavlink stream -r 50 -s ATTITUDE_QUATERNION -u 14556
mavlink stream -r 50 -s ATTITUDE_TARGET -u 14556
mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u 14556
mavlink stream -r 20 -s RC_CHANNELS -u 14556
mavlink stream -r 250 -s HIGHRES_IMU -u 14556
mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u 14556
 ```
 ## for sitl & mavros connection
 startup file
 ```
 param set MAV_SYS_ID 1
 param set SITL_UDP_PRT 14560
 mavlink start -x -u 14557 -r 4000000 -m onboard -o 14540
 ```
 and launch file lines
```xml
<arg name="tgt_system" value="$(arg ID)"/>
<arg name="mavlink_udp_port" value="14560"/>
<arg name="fcu_url" default="udp://:14540@localhost:14557"/>
```
should be matched
