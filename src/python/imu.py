import serial
import time
import rospy
from std_msgs.msg import Float64MultiArray
 
ser_imu = serial.Serial('/dev/ttyUSB1', 
                    baudrate = 921600, 
                    parity = serial.PARITY_NONE,    
                    stopbits = serial.STOPBITS_ONE, 
                    bytesize = serial.EIGHTBITS)
ser_stm = serial.Serial('/dev/ttyUSB2', 
                    baudrate = 115200, 
                    parity = serial.PARITY_NONE,    
                    stopbits = serial.STOPBITS_ONE, 
                    bytesize = serial.EIGHTBITS)

#data_config = ser_stm.readline()
#ser_imu.write(data_config)

node_name = 'Serial_Port'
rospy.init_node(node_name)
pub = rospy.Publisher('PCDAT', Float64MultiArray, queue_size=10)

while True:
    data_imu = ser_imu.readline()      
    data_imu = '\n' + data_imu
    print(data_imu)
    ser_stm.write(data_imu) 
    

ser.close()
