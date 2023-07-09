## @package rt2_assignment1
# \file user_interface_action.py
# \brief This file contains code for 'user_interface' node.
# \author Shozab Abidi
# \version 1.0
# \date 25/10/2021
#
# \details
#
# Subscribes : <BR>
# 	° /user_interface_sig
#
# Service : <BR>
# ° /user_interface
#
# This node is user interface of a 2D robot simulation project.
#

import rospy
import time
from rt2_assignment1.srv import Command
from std_msgs.msg import String


##  A global variable with datatype 'string' used as a flag to start the simulation.
value_ = "-1"  
## A Global variable with datatype 'string' used as a flag to control the conditions for 'while' loop.           
simulation_ = "off"    

##
# \brief This is a callback function of the subscriber for the ROS topic '/user_interface_sig'. 
# \param msg is the string data  
# \return [none].
#
# This is function takes the msg data coming from the  '/user_interface_sig' topic and 
# store it in the global variable 'value_.' 
#
def clbk_user_interface(msg):
	global value_
	value_ = msg.data

##
# \brief This is a 'main' function of user_interface node. 
# 
# \return [none].
#
# This function is a 'main' function of  'user_interface' node. It initializes client for '/user_interface'
# service hosted by 'state_machine' node and subscriber for the 'user_interface_sig' topic. Upon 
# user's request it send the signal to the 'state_machine' node to state the 'random target position
# 'state simulation. 
#
def main():

	global simulation_
	rospy.init_node('user_interface')
	ui_client = rospy.ServiceProxy('/user_interface', Command)
	user_interface_sig = rospy.Subscriber('/user_interface_sig', String, clbk_user_interface)
	time.sleep(5)
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if(simulation_ == "off" and value_ == "1"):
			ui_client("start")
			simulation_ = "on"
		elif(simulation_ == "on" and value_ == "0"):
			ui_client("stop")
			simulation_ = "off"

 
if __name__ == '__main__':
	main()
