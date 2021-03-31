Running PX4 in SITL Mode
==========================

Steps
---------------------------

1. Download PX4 v1.11.3 Firmware
```
git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.11.3  
cd PX4-Autopilot
git submodule update --init --recursive
sudo ./Tools/setup/ubuntu.sh
make px4_sitl_default
make px4_sitl_default gazebo
```

2. Source your environment:
Add the following code to the bottom of '~/.bashrc '.
Here, ... must be modified to suit your own path.
```bash
source ~/.../PX4-Autopilot/Tools/setup_gazebo.bash ~/.../PX4-Autopilot ~/.../PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/.../PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/.../PX4-Autopilot/Tools/sitl_gazebo
```
3. test launch
```
roslaunch px4 mavros_posix_sitl.launch
```

or

```
roslaunch px4 multi_uav_mavros_sitl.launch
```

and then
```
roslaunch swarm_ctrl_pkg mavros_sitl_with_swarm_ctrl.launch
```

Now you have a drone that you can control from another terminal.
Enter the following commands in order
```
goto 0 0 5
arm
offboard
goto 0 5 5
```
To use the above commands, goto, arm, and offboard must be defined in ~/.bashrc.

Detailed settings for multi_UAV gazebo
---------------------------
군집 드론의 시뮬레이션을 이용하기 위해서는 시뮬레이션에서 사용되는 모든 udp, tcp 포트가 각각의 드론에 맞게 설정되어야 한다.
이러한 설정은 `px4/ROMFS/px4fmu_common/init.d-posix/rcS`에 109~116번째 줄에 표기되어 있다. 아래는 해당 코드이다 
```
param set MAV_SYS_ID $((px4_instance+1))
simulator_tcp_port=$((4760+px4_instance))
udp_offboard_port_local=$((14580+px4_instance))
udp_offboard_port_remote=$((14840+px4_instance))
# [ $px4_instance -gt 9 ] && udp_offboard_port_remote=14549 # use the same ports for more than 10 instances to avoid port overlaps
udp_onboard_payload_port_local=$((14280+px4_instance))
udp_onboard_payload_port_remote=$((14030+px4_instance))
udp_gcs_port_local=$((18570+px4_instance))
```
여기서 우리는 `simulation_tcp_port`, `udp_offboard_port_local`, `udp_offboard_port_remote` 세가지의 숫자가 중요하다

위의 세가지 파라미터 숫자는 `swarm_ctrl_pkg/launch/mavros_sitl_swarm_ctrl.launch`에서 35~47번째 줄의 코드인 아래 코드에서 확인 가능하다. 
```xml
<arg name="ID" value="0"/>
        <arg name="fcu_url" default="udp://:14840@localhost:14580"/>
        <!-- PX4 SITL and vehicle spawn -->
        <include file="$(find px4)/launch/single_vehicle_spawn.launch">
            <arg name="x" value="0"/>
            <arg name="y" value="$(arg ID)"/>
            <arg name="z" value="0"/>
            <arg name="R" value="0"/>
            <arg name="P" value="0"/>
            <arg name="Y" value="0"/>
            <!-- <arg name="est" value="$(arg est)"/> -->
            <arg name="vehicle" value="$(arg vehicle)"/> 
            <arg name="mavlink_udp_port" value="14760"/>
            <arg name="mavlink_tcp_port" value="4760"/>
```
`udp_offboard_port_local`과 `udp_offboard_port_remote`는 위 코드이 `fcu_url`에서 사용되며, `simulation_tcp_port`는 `mavlink_tcp_port`로 사용된다.



#available port number
```
 포트는 받는놈 기준
```
ex) mavros 의 14557 이 받는다
|        |                     |        |
| :----: | :-----------------: | :----: |
|        | ---  `-u 14557` --> |        |
| px4    |                     | mavros |
|        | <--  `-o 14540` --- |        |


px4 >> -u 14557 >> mavros


| Number | sitl_udp_port | -u    | -o    | Qgc udp | Qgc remote |
| :----: | :-----------: | :---: | :---: | :-----: | :--------: |
| 1      | 13001         | 13101 | 13201 | 13301   | 13401      |
| 2      | 13002         | 13102 | 13202 | 13302   | 13402      |
| 3      | 13003         | 13103 | 13203 | 13303   | 13403      |
| 4      | 13004         | 13104 | 13204 | 13304   | 13404      |
| 5      | 13005         | 13105 | 13205 | 13305   | 13405      |
| 6      | 13006         | 13106 | 13206 | 13306   | 13406      |
| 7      | 13007         | 13107 | 13207 | 13307   | 13407      |
| 8      | 13008         | 13108 | 13208 | 13308   | 13408      |
| 9      | 13009         | 13109 | 13209 | 13309   | 13409      |
| 10     | 13010         | 13110 | 13210 | 13310   | 13410      |
| 11     | 13011         | 13111 | 13211 | 13311   | 13411      |
| 12     | 13012         | 13112 | 13212 | 13312   | 13412      |
| 13     | 13013         | 13113 | 13213 | 13313   | 13413      |
| 14     | 13014         | 13114 | 13214 | 13314   | 13414      |
| 15     | 13015         | 13115 | 13215 | 13315   | 13415      |
| 16     | 13016         | 13116 | 13216 | 13316   | 13416      |
| 17     | 13017         | 13117 | 13217 | 13317   | 13417      |
| 18     | 13018         | 13118 | 13218 | 13318   | 13418      |
| 19     | 13019         | 13119 | 13219 | 13319   | 13419      |
| 20     | 13020         | 13120 | 13220 | 13320   | 13420      |
| 21     | 13021         | 13121 | 13221 | 13321   | 13421      |
| 22     | 13022         | 13122 | 13222 | 13322   | 13422      |
| 23     | 13023         | 13123 | 13223 | 13323   | 13423      |
| 24     | 13024         | 13124 | 13224 | 13324   | 13424      |
| 25     | 13025         | 13125 | 13225 | 13325   | 13425      |
| 26     | 13026         | 13126 | 13226 | 13326   | 13426      |
| 27     | 13027         | 13127 | 13227 | 13327   | 13427      |
| 28     | 13028         | 13128 | 13228 | 13328   | 13428      |
| 29     | 13029         | 13129 | 13229 | 13329   | 13429      |
| 30     | 13030         | 13130 | 13230 | 13330   | 13430      |
| 31     | 13031         | 13131 | 13231 | 13331   | 13431      |
| 32     | 13032         | 13132 | 13232 | 13332   | 13432      |
| 33     | 13033         | 13133 | 13233 | 13333   | 13433      |
| 34     | 13034         | 13134 | 13234 | 13334   | 13434      |
| 35     | 13035         | 13135 | 13235 | 13335   | 13435      |
| 36     | 13036         | 13136 | 13236 | 13336   | 13436      |
| 37     | 13037         | 13137 | 13237 | 13337   | 13437      |
| 38     | 13038         | 13138 | 13238 | 13338   | 13438      |
| 39     | 13039         | 13139 | 13239 | 13339   | 13439      |
| 40     | 13040         | 13140 | 13240 | 13340   | 13440      |
| 41     | 13041         | 13141 | 13241 | 13341   | 13441      |
| 42     | 13042         | 13142 | 13242 | 13342   | 13442      |
| 43     | 13043         | 13143 | 13243 | 13343   | 13443      |
| 44     | 13044         | 13144 | 13244 | 13344   | 13444      |
| 45     | 13045         | 13145 | 13245 | 13345   | 13445      |
| 46     | 13046         | 13146 | 13246 | 13346   | 13446      |
| 47     | 13047         | 13147 | 13247 | 13347   | 13447      |
| 48     | 13048         | 13148 | 13248 | 13348   | 13448      |
| 49     | 13049         | 13149 | 13249 | 13349   | 13449      |
| 50     | 13050         | 13150 | 13250 | 13350   | 13450      |

