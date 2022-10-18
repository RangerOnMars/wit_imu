# Wit IMU Node
# Author: RangerOnMars guhao0521@gmail.com
## 1.Introduction
A simple node written in Python to transform Data from WIT IMU into standard sensors_msgs/IMU format and publish the message.
## 2.Parameters
 `port_id`:Com ID string in format "VID:PID",must written in lower.e.g."1a86:7523"   
 `frame_id`: Frame ID of your IMU data.  
 `imu_topic`: Topic where you want to publish these messages.   
 `serial_baudrate`: Baudrate of your IMU.   