# Template for ROS-Arduino control architecture

1) read joystick buttons to use as a set-point
2) compute command in a python controller script
3) command is send to an arduino using rosserial
4) low-level task executed on the arduino
5) sensor data from the arduino is sent back to the python controller using rosserial

![Screenshot from 2021-06-15 22-34-37](https://user-images.githubusercontent.com/16725496/122156256-a236fa80-ce36-11eb-9b16-70147ef95720.png)

![IMG_0799](https://user-images.githubusercontent.com/16725496/122602790-66bc4c00-d041-11eb-8b34-188841b2bc51.jpg)
