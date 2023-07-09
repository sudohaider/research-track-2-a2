#! /usr/bin/env python

## @package rt2_assignment1
# \file go_to_point_action.py
# \brief This file contains code for 'go_to_point' node.
# \author Muhammad Ali Haider Dar
# \version 1.0
# \date 20/05/2023
#
# \details
#
# Subscribes : <BR>
# 	° /odom
# 	° /control_speed
#
# Publishers : <BR>
# 	° /odom
# 	° /control_speed
# 	° /cancel_goals
#   ° /success_goals
#
# Service : <BR>
#   ° /go_to_point
#
# This node receives the desired goal coordinates as an action service '/go_to_point' request from the 'state_machine' node. Based on the received goal coordinates, it computes the required linear and angular velocities of the robot to reach the target position and then publishes it on topic 'cmd_vel'. Since "/go_to_point" is an action service, it offers the option to cancel the goal at any point during the execution. For this purpose, the goal preempt requested scheme has also been implemented which gets activated when user cancel any goal. Within this scheme, number of cancelled goals are also accumulated.
#



import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment1.srv import Position

from std_msgs.msg import String, Int32MultiArray
import math
import actionlib
import rt2_assignment1.msg
import time

# robot state variables

##  A global variable with data structure type 'Point' used for storing the current position of robot.
position_ = Point() 
##  A global variable for storing robot's yaw value.                         
yaw_ = 0
##  Initializing global variable 'position_' with 0.
position_ = 0
##  A global variable for storing the linear speed factor.
speed_factor_linear_ = 0
##  A global variable for storing the angular speed factor.
speed_factor_angular_ = 0
##  A global variable for initializing publisher for topic '/cmd_vel'.
pub_ = None
##  A global variable for initializing publisher for topic '/success_goals'.
pub_succ_status_ = None
##  A global variable for initializing publisher for topic '/cancel_goals'.
pub_cancel_status_ = None
##  A global variable for initializing publisher for topic '/task_time'.
pub_task_time_ = None
##  A global variable for storing the total number of successful goals.
no_successful_target_ = 0
##  A global variable for storing the total number of cancelled  goals.
no_cancelled_target_ = 0
##  A global variable with data structure type 'Int32MultiArray()' for storing the time taken by the robot to complete the tasks.
t_array = Int32MultiArray()

# parameters for control

##  A global variable with datatype float, used as a parameter for robot's yaw precision.
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
##  A global variable with datatype float, also used as a parameter for robot's yaw precision.
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
##  A global variable with datatype float used as a parameter for robot's distance precision.
dist_precision_ = 0.1
##  A global variable used as a parameter for generating robot's new velocities.
kp_a = -3.0 
##  A global variable used as a parameter for generating robot's new velocities.
kp_d = 0.2
##  A global variable used as a parameter for generating robot's new velocities.
ub_a = 0.6
##  A global variable used as a parameter for generating robot's new velocities.
lb_a = -0.5
##  A global variable used as a parameter for generating robot's new velocities.
ub_d = 0.6


##
# \brief This is a callback function of the subscriber 'sub_odom' for the ROS topic '/odom'
# \param msg argument with data structure type Odometry  
# \return [none]
#
# This function takes the robot's odometry data coming from the '/odom' topic, 
# extract the position and yaw and then store it in the global variables 'position_' 
# and 'yaw_'. 
#
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]



##
# \brief This is a callback function of the subscriber 'ctrl_speed' for the ROS topic '/odom' 
# \param msg argument with data structure type 'Twist '  
# \return [none]
#
# This function takes the velocity factors value coming from the '/control_speed' topic
# and then store them in the global variables 'speed_factor_linear_' and 'speed_factor_angular_.  
#
def clbk_ctrl_speed(msg):

	global speed_factor_linear_
	global speed_factor_angular_
	
	speed_factor_linear_ = msg.linear.x
	speed_factor_angular_ = msg.angular.z
	
	
##
# \brief This is a function used for changing the value of global variable 'state_'. 
# \param state argument with data type 'int '  
# \return [none].
#
# This function takes the argument value and assigned it to the global variable 'state_'. 
#
def change_state(state):
    global state_
    state_ = state
    #rospy.loginfo('State changed to [%s]' % state_)


##
# \brief This is a function used for normalizing the orientation angle of robot from goal orientation. 
# \param angle argument with data type 'float'  
# \return Normalized angle value.
#
# This is a function used for normalizing the orientation angle of robot from goal orientation. 
#

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
# \brief This function adjust the yaw of the robot. 
# \param des_pos argument with datatype 'float'.  
# \return [none].
#
# This function takes the desired position data as argument. Based on the 'yaw error' calculates the
# required angular velocity and then publishes it in the topic 'cmd_vel'. 
#
def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a + ub_a*speed_factor_angular_  # I make changes here
        elif twist_msg.angular.z < lb_a: 
            twist_msg.angular.z = lb_a + lb_a*speed_factor_angular_   # I make changes here
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
    	#print ('Yaw error: [%s]' % err_yaw)
    	change_state(1)
        
##
# \brief This function calculates the robot's linear velocity. 
# \param des_pos argument with datatype 'float'.  
# \return [none].
#
# This function takes the desired position data as argument. Calculate the 'desired yaw', 'yaw error'
# and 'position error'  and based on these values calculates the required linear velocity and then 
# publishes it in the topic 'cmd_vel'. 
#
def go_straight_ahead(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3 + 0.3*speed_factor_linear_
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        change_state(0)

##
# \brief This function adjust the final yaw of the robot. 
# \param des_yaw argument with datatype 'float'.  
# \return [none].
#
# This function takes the desired yaw data as argument. Based on the 'yaw error' calculates the
# final angular velocity require to match goal orientation and then publishes it in the topic 'cmd_vel'. 
#
def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    #rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a + ub_a*speed_factor_angular_    # I make changes here
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a  +  lb_a*speed_factor_angular_  # I make changes here
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
##
# \brief This is function makes the linear and angular velocities of robot to zero. 
# \param [none]   
# \return [none].
#
# This is function makes the linear and angular velocities to zero. Generally it use when robot
# has successfully reach the target or when user have requested it to stop.  
#
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
   
   
##
# \class PositionAction
# \brief This class defines the 'go_to_point' action service through which 'state_machine' node assign 
# it goals.
# 
# This class defines the 'go_to_point' action service through which 'state_machine' node assign 
# it goals. It also controls the internal state_machine of 'go_to_point' node which drives the robot from
# current position to goal position. During the simulation based on the robot state it generates and 
# publish feedbacks and insights which is use by jupyter notebook's user interface script.  
#

    
class PositionAction(object):
    # create messages that are used to publish feedback/result
    _feedback = rt2_assignment1.msg.PositionFeedback()
    _result = rt2_assignment1.msg.PositionResult()

	##	
	# \brief This is a constructor function of the class. 
	# \param name argument of type 'String'  
	# \return [none].
	#
	# This is function defines the action service 'go_to_point'. 
	#
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rt2_assignment1.msg.PositionAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    ##
	# \brief This is a callback function for the service 'go_to_point'. 
	# \param goal argument with custom define data structure type 'Position'. 
	# \return trure always.	
	#
	# This function execute when request is recieved from the client with argument goal position. 
	# After recieving the goal position it starts the internal state mahcine for driving the robot. This 
	# function also updates and publishes some global variables which highlight some key insights such as number successful and cancelled goals, 
	# and list of all the recordded time taken by the robot to reach targets.
	#
    def execute_cb(self, goal):
    	# helper variables
    	r = rospy.Rate(1)
    	success = False
    	#go_to_point(goal)
    	#feedback = Sim()
    	global no_successful_target_
    	global no_cancelled_target_
    	global t_array	

    	desired_position = Point()
    	desired_position.x = goal.x
    	desired_position.y = goal.y
    	des_yaw = goal.theta
    	change_state(0)
    	# For calculating the time taken by the robot to reach the desired goal. 
    	start = time.time()
    	
    	#rospy.loginfo('%s: Executing, following target positions x = %i, y = %i, theta = %i' % (self._action_name, desired_position.x, desired_position.y,des_yaw))
    	while True:
    		# check that preempt has not been requested by the client.
    		if self._as.is_preempt_requested():
    			rospy.loginfo('%s: Preempted' % self._action_name)
    			self._as.set_preempted()
    			
    			# For Bar Graph in Jupyter Notebook
    			no_cancelled_target_ +=  1
    			print("No of Cancel goals : %d", no_cancelled_target_)
    			pub_cancel_status_.publish(str(no_cancelled_target_))
    			success = False
    			done()
    			break
    		if state_ == 0:
    			self._feedback.ok = True
    			self._as.publish_feedback(self._feedback)
    			#rospy.loginfo("Fixing in the yaw.")
    			fix_yaw(desired_position)
    		elif state_ == 1:
    			self._feedback.ok = True
    			self._as.publish_feedback(self._feedback)
    			#rospy.loginfo("going straight ahead.")
    			go_straight_ahead(desired_position)
    		elif state_ == 2:
    			self._feedback.ok = True
    			#rospy.loginfo("Fixing the final yaw.")
    			self._as.publish_feedback(self._feedback)
    			fix_final_yaw(des_yaw)
    		elif state_ == 3:
    			self._feedback.ok = True
    			rospy.loginfo('%s: Succeeded' % self._action_name)
    			self._as.set_succeeded(self._result)
    			no_successful_target_ += 1
    			print("No of Successful goals : ", no_successful_target_)
    			pub_succ_status_.publish(str(no_successful_target_))
    			
    			# For Time Histogram in Jupyter Notebook
    			
    			end = time.time()
    			t = math.floor(end - start)
    			t_array.data.append(t)
    			pub_task_time_.publish(t_array)
    			print("Time taken to reach the task : ", t_array)
    			
    			done()
    			break
    	return True     

##
# \brief This is a main function of the ROS node 'go_to_point'. 
# \param [none]  
# \return [none].
#
# This function is a 'main' function of  'go_to_point' node. It initializes the subscribers for '/odom' 
# and '/control_speed' topics, publishers for '/cmd_vel', 'success_goals', '/cancel_goal', 'task_time'
# topics and lastly call the PositionAction service.  
#
def main():
    global pub_
    global pub_succ_status_
    global pub_cancel_status_
    global pub_task_time_
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_succ_status_ = rospy.Publisher('/success_goals', String, queue_size=1)
    pub_cancel_status_ = rospy.Publisher('/cancel_goals', String, queue_size=1)
    pub_task_time_ = rospy.Publisher('/task_time', Int32MultiArray, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    ctrl_speed = rospy.Subscriber('/control_speed', Twist, clbk_ctrl_speed)
    server = PositionAction(rospy.get_name())
    rospy.spin()

if __name__ == '__main__':
    main()
