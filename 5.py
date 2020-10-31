import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math


kp = 1

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------
def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw

# CALLBACKS ---------------------------------------------------------
def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg
#--------------------------------------------------------------------

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
	sp=0
	state=1
	control=0
	if len(scan.ranges) > 0:
		scanmin=min(scan.ranges)
		for i in range(len(scan.ranges)):
			if scan.ranges[i] == scanmin:
				sp=i
		print("scanmin:")
		print(scanmin)
		print("Len:")
		print(len(scan.ranges))
		print("sp:")
		print(sp)
		print("angmin:")
		print(scan.angle_min)
		print("angmax:")
		print(scan.angle_max)

	#Estado 2,virando para o setpoint
	yaw = getAngle(odom) 
	setpoint = sp
	error = (setpoint - yaw)
    
	if abs(error) > 180:
		if setpoint < 0:
			error += 360 
		else:
			error -= 360
        
		P = kp*error
		I = 0
		D = 0
		control = P+I+D

	msg = Twist()
	msg.angular.z = control
	pub.publish(msg)


pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(0.05), timerCallBack)

rospy.spin()