#!/usr/bin/env python

import rospy as rp
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
import math
from std_msgs.msg import String

rp.init_node('node1')
pub = rp.Publisher('Matricula', String, queue_size=20)

def timerCallBack(event):
	msg = "2017001339"
    	pub.publish(msg)

def Soma_callBack(msg):
    global Soma
    Soma = msg
    print(Soma)

timer = rp.Timer(rp.Duration(0.1),timerCallBack)
sub = rp.Subscriber('/Soma',String, Soma_callBack)


rp.spin()
