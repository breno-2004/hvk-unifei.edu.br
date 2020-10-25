#!/usr/bin/env python

import rospy as rp
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
import math
from std_msgs.msg import String
rp.init_node('node2')
pub = rp.Publisher('Soma', String, queue_size=20)
global Matricula
global Soma
def Matricula_callBack(msg):
    global Matricula
    Matricula = msg

def sum_digits_string(str1):
    sum_digit = 0
    for x in str1:
        if x.isdigit() == True:
            z = int(x)
            sum_digit = sum_digit + z
    return sum_digit

def timerCallBack(event):
	global Matricula
	global Soma
	Soma=sum_digits_string(str(Matricula))
	print(Matricula)
    	pub.publish(str(Soma))

timer = rp.Timer(rp.Duration(0.1),timerCallBack)
sub = rp.Subscriber('/Matricula',String, Matricula_callBack)


rp.spin()
