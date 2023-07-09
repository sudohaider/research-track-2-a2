/**
* \file state_machine_action.cpp
* \brief This files contains code for the 'state_machine' node.
* \author Shozab Abidi
* \version 1.0
* \date 25/10/2021
*
* \details
* 
* Services : <BR>
* ° /user_interface
* ° /position_server
* 
* Action Clients / Services : <BR>
* ° /PositionAction
*  
* Description :
*
* This node implements the state machine of the project. When the user starts the simulation, it receives a 'string input' as request to the service '/user interface' from     * 'user_interface_action' node. The 'string input' value act as a flag to signal the node to start the state machine loop in which it request another service '/position_server' * which is hosted by 'position_service' node to randomly generate goal coordinates for the robot to follow. Once it receives the goal coordinates in response to the earlier * request to the service, it pass these goal coordinates to an action service '/go_to_point' which is host by 'go_to_point_action.py' node. 
*
*/

#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/PositionAction.h>
#include <typeinfo>
#include <string>


///< Global variable used as a flag to start the simulation.
int start_ = -1;
///< Global variable used to control the state machine loop.                            
int count_ = 0;
///< Global variable used to store the state of assigned action.                           
std::string Check_ = "non";        


/**
* \brief A callback function of the client for the 'user_interface'
* \param req request arguement of the service 'user_interface' with data type Command.Requesy 
* \param res response arguement of the service 'user_interface' with data type Command.Response
* \return always true as this method cannot fail.
*
* This function change the flag variable 'start_'  value according to the user's instruction which will either start or stop the simulation.
* 
*/
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res)
{
    if (req.command == "start")
    {
    	start_ = 1;
    	count_ = 1;
    }
    else
    {
    	start_ = 0;
    }
    return true;
}

typedef actionlib::SimpleActionClient<rt2_assignment1::PositionAction> StateMachineClient;

/**
* \brief The main function of the node 'state_machine'
* \param argc an integer arguement  
* \param an string double pointer arguement.
*
* \return always return 0 as this function cannot fail.
*
* This function initialize the ros node 'state_machine', client for service '/position_server', action client for action service '/go_to_point', advertise the service '/user             * interface' and implement state machine loop for the simulation. 
* 
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   
   // Telling the action clinet that we want to spin the thread by default.
   StateMachineClient ac("/go_to_point", true);
  
    
   ROS_INFO("Waiting for action server to start.");
                                              // wait for the action server to start
   ac.waitForServer();                        //will wait for infinite time

   ROS_INFO("Action server started, sending goal."); 
    
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   
   rt2_assignment1::PositionGoal goal;
   
   while(ros::ok())
   {
   		ros::spinOnce();
   		actionlib::SimpleClientGoalState state = ac.getState();
   	
	   	if(start_ == 1)
	   	{
	   	    if(count_ == 1)
		    {
		    	client_rp.call(rp);
	   		goal.x = rp.response.x;
	   		goal.y = rp.response.y;
	   	        goal.theta = rp.response.theta;  //this goal is keeps on changing because while is active.
		        ac.sendGoal(goal);
		        count_++;
		        std::cout << "\nGoing to the position: x= " << goal.x << " y= " <<goal.y << " theta = " <<goal.theta << std::endl;
		    } 
	   	    
	   	    Check_ = state.toString().c_str();
	   	    
	   	    if(Check_ == "SUCCEEDED")
	   	    {
	   	        std::cout << "\nPosition Reached! " << std::endl;  
	   	        client_rp.call(rp);
	   	        goal.x = rp.response.x;
		        goal.y = rp.response.y;
		        goal.theta = rp.response.theta;  //this goal is keeps on changing because while is active.       
		        ac.sendGoal(goal);
		        std::cout << "\nGoing to the position: x= " << goal.x << " y= " <<goal.y << " theta = " <<goal.theta << std::endl;
		    }   
		    
		}
	   	else if(start_ == 0)
	   	{
	   	    ac.cancelAllGoals();
	   	    std::cout << "Current Goal is Cancelled!" << std::endl;
	   	    count_ = 0;
	   	    start_ = -1;
	   	}
   }  	
  
   
   return 0;
}
