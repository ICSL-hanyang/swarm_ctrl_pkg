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


  
