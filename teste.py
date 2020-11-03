import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math


kp = 0.01
ki =0.01
kd = 0.01
I = 0
setpoint = 0
error = 0
old_error = 0

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
    global kp
    global ki
    global I
    global kd
    global setpoint
    global error
    global old_error
	
    yaw = getAngle(odom)
    setpoint = 90
    error = (setpoint - yaw)
    print(error)
    if abs(error) > 180:
        if setpoint < 0:
            error += 360 
        else:
            error -= 360
	
    P = kp*error
    I = (I + error) * ki
    #I = 0
    D = (error - old_error)*kd
    #D = 0

    PID = P + I + D
    print(PID)
    old_error = error
    #error = old_error
    msg = Twist()
    msg.angular.z = PID
    pub.publish(msg)

    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(0.05), timerCallBack)

rospy.spin()
