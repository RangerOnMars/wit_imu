from json import tool
from serial.tools import list_ports

import serial
import rospy
from sensor_msgs.msg import Imu 
from tf.transformations import quaternion_from_euler

ACCData=[0.0]*8
GYROData=[0.0]*8
AngleData=[0.0]*8          
FrameState = 0            #通过0x后面的值判断属于哪一种情况
Bytenum = 0               #读取到这一段的第几位
CheckSum = 0              #求和校验位         
 
a = [0.0]*3
w = [0.0]*3
Angle = [0.0]*3

def getComByID(str):
    """A function to get Com directory by its ID.

    Args:
        str (_type_): Com ID string in format "VID:PID",must written in lower.

    Raises:
        KeyError: _description_

    Returns:
        _type_: Com directory string.
    """
    assert len(str) == 9 and str[4] == ":", "Illegal ID String"
    # 获取端口列表，列表中为 ListPortInfo 对象
    port_list = list(list_ports.comports())

    num = len(port_list)
    assert len(str)
    assert num > 0, "The number of comports is less than one."

    for i in range(num):
        #Check USB VID And PID of the device
        if port_list[i].vid == int(str[0:4], 16) and port_list[i].pid == int(str[5:], 16):
            return port_list[i].device
    
    raise KeyError("Comport with selected ID does not exist.")

def DueData(inputdata):   #新增的核心程序，对读取的数据进行划分，各自读到对应的数组里
    global  FrameState    #在局部修改全局变量，要进行global的定义
    global  Bytenum
    global  CheckSum
    global  a
    global  w
    global  Angle
    d = ()
    get_valid = False
    # print(inputdata[0])
    for data in inputdata:  #在输入的数据进行遍历
        #Python2软件版本这里需要插入 data = ord(data)*****************************************************************************************************
        if FrameState == 0:   #当未确定状态的时候，进入以下判断
            if data == 0x55 and Bytenum == 0: #0x55位于第一位时候，开始读取数据，增大bytenum
                CheckSum = data
                Bytenum = 1
                continue
            elif data == 0x51 and Bytenum == 1:#在byte不为0 且 识别到 0x51 的时候，改变frame
                CheckSum += data
                FrameState = 1
                Bytenum = 2
            elif data == 0x52 and Bytenum == 1: #同理
                CheckSum += data
                FrameState = 2
                Bytenum = 2
            elif data == 0x53 and Bytenum == 1:
                CheckSum += data
                FrameState = 3
                Bytenum = 2

        elif FrameState == 1: # acc    #已确定数据代表加速度
            if Bytenum < 10:            # 读取8个数据
                ACCData[Bytenum-2] = data # 从0开始
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum&0xff):  #假如校验位正确
                    a = get_acc(ACCData)
                CheckSum=0                  #各数据归零，进行新的循环判断
                Bytenum=0
                FrameState=0

        elif FrameState == 2: # gyro
            if Bytenum < 10:
                GYROData[Bytenum-2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum&0xff):
                    w = get_gyro(GYROData)
                    d = a + w
                    get_valid = True
                CheckSum = 0
                Bytenum = 0
                FrameState = 0
        elif FrameState == 3: # angle
            if Bytenum < 10:
                AngleData[Bytenum-2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum&0xff):
                    Angle = get_angle(AngleData)
                    d = a + w + Angle
                    get_valid = True
                    CheckSum = 0
                    Bytenum = 0
                    FrameState = 0
                    return d, get_valid
                CheckSum = 0
                Bytenum = 0
                FrameState = 0
    
 
 
def get_acc(datahex):  
    axl = datahex[0]                                        
    axh = datahex[1]
    ayl = datahex[2]                                        
    ayh = datahex[3]
    azl = datahex[4]                                        
    azh = datahex[5]
    
    k_acc = 16.0 * 9.8035
 
    acc_x = (axh << 8 | axl) / 32768.0 * k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
    acc_z = (azh << 8 | azl) / 32768.0 * k_acc
    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z-= 2 * k_acc
    
    return acc_x,acc_y,acc_z
 
 
def get_gyro(datahex):                                      
    wxl = datahex[0]                                        
    wxh = datahex[1]
    wyl = datahex[2]                                        
    wyh = datahex[3]
    wzl = datahex[4]                                        
    wzh = datahex[5]
    k_gyro = 2000.0
 
    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >=k_gyro:
        gyro_z-= 2 * k_gyro
    return gyro_x,gyro_y,gyro_z
 
 
def get_angle(datahex):                                 
    rxl = datahex[0]                                        
    rxh = datahex[1]
    ryl = datahex[2]                                        
    ryh = datahex[3]
    rzl = datahex[4]                                        
    rzh = datahex[5]
    k_angle = 180.0
 
    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >=k_angle:
        angle_z-= 2 * k_angle
 
    return angle_x,angle_y,angle_z
 
 
if __name__=='__main__':
    rospy.init_node('wit_imu', anonymous=True)
    port_id = rospy.get_param("/wit_imu/port_id")
    frame_id = rospy.get_param("/wit_imu/frame_id")
    brate = rospy.get_param("/wit_imu/serial_baudrate")
    topic = rospy.get_param("/wit_imu/imu_topic")
    pub = rospy.Publisher(topic,Imu,queue_size=50)
    
    port = ""
    while True:
        try:
            port = getComByID(port_id)
            break
        except AssertionError:
            print("Illegal ID String or The number of comports is less than one.")
        except KeyError:
            print("Comport with selected ID does not exist.")
        
    ser = serial.Serial(port, brate, timeout=None)
    
    
    if (ser.is_open):
        #Main Loop
        while(not rospy.is_shutdown()):
            ser.flush()
            datahex = ser.read(44)
            data, data_received = DueData(datahex)
            rate = rospy.Rate(100) # 400hz
            if (data_received):
                # print("a(g):%10.3f %10.3f %10.3f w(deg/s):%10.3f %10.3f %10.3f"%data[0:6])
                # print("Roll : {} Pitch : {} Yaw : {}".format(data[6],data[7],data[8]))
                imu = Imu()
                imu.header.stamp = rospy.Time.now();
                imu.header.frame_id = frame_id
                imu.linear_acceleration.x = data[0]
                imu.linear_acceleration.y = data[1]
                imu.linear_acceleration.z = data[2]
                imu.angular_velocity.x = data[3] / 180.0 * 3.1415926
                imu.angular_velocity.y = data[4] / 180.0 * 3.1415926
                imu.angular_velocity.z = data[5] / 180.0 * 3.1415926
                q = quaternion_from_euler(data[6] / 180.0 * 3.1415926, data[7] / 180.0 * 3.1415926, data[8] / 180.0 * 3.1415926)
                imu.orientation.x = q[0]
                imu.orientation.y = q[1]
                imu.orientation.z = q[2]
                imu.orientation.w = q[3]

                # print("Done!")
                pub.publish(imu)
                rate.sleep()



    
