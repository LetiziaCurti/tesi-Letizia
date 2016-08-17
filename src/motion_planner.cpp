// Nodo che modella il motion planner


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"
#include "task_assign/vect_robot.h"
#include "task_assign/rt_vect.h"
#include "task_assign/vect_info.h"
#include "task_assign/path.h"


inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}



using namespace std;
ros::Subscriber obs_sub;
ros::Subscriber assignment_sub;
ros::Subscriber status_rob_sub;
ros::Publisher exec_task_pub;
ros::Publisher rob_ass_pub;
ros::Publisher rob_ini_pub;
ros::Publisher rob_info_pub;
ros::Publisher rt_pub;



vector<task_assign::robot> available_robots;    	
vector<task_assign::robot> not_available_robots;    	



// Legge "obstacles_topic" 
void ObsCallback(const task_assign::vect_task::ConstPtr& msg)
{

}


// Legge "assignment_topic" 
void AssignCallback(const task_assign::rt_vect::ConstPtr& msg)
{

}


// Legge "status_rob_topic" 
void StatusCallback(const task_assign::robot::ConstPtr& msg)
{
    if(msg->status)
    {
	bool add_rob(true);
    
	for(auto elem : available_robots)
	{
	    if(elem.name == msg->name)
	    {
		add_rob = false;
		break;
	    }
	}
	
	if(add_rob)
	{
	    for(auto elem : not_available_robots)
	    {
		if(elem.name == msg->name)
		{
		    add_rob = false;
		    
		    //aggiorna la posizione
		    
		    break;
		}
	    }
	}
   
	if(add_rob)
	    available_robots.push_back(msg);

    }
}


// Pubblica al master su "rob_assign_topic il vettore dei robot da assegnare
void publishRobotToAssign()
{
    task_assign::vect_robot vect_msg;
}



// Pubblica al master su "rob_info_topic" le info relative ai robot (tutti: già assegnati e da assegnare)
void publishRobotIni()
{
    task_assign::vect_robot vect_msg;
    
    // deve monitorare il b_lev, se scende al di sotto della soglia il robot deve essere tolto dal vettore
    
//     if(t_ex > t_ex_0)
// 		b_lev += (t_ex - t_ex_0)*1;
// 	    else
// 		b_lev++;
// 	    
// 	    robot.b_level = b_lev;
}




// Pubblica al master su "rob_info_topic" le info relative ai robot (tutti: già assegnati e da assegnare)
void publishRobotInfo()
{
    task_assign::vect_robot vect_msg;
    
    // deve monitorare il b_lev, se scende al di sotto della soglia il robot deve essere tolto dal vettore
    
//     if(t_ex > t_ex_0)
// 		b_lev += (t_ex - t_ex_0)*1;
// 	    else
// 		b_lev++;
// 	    
// 	    robot.b_level = b_lev;
}


// Function che elimina il robot con il nome passato in argomento dal vettore di robot passato in argomento
vector<task_assign::robot> deleteElem(string name, vector<task_assign::robot> vect)
{
    int i(0);
    for(auto elem : vect)
    {
	if(elem.name == name)
	{
	    vect.erase(vect.begin()+i);
	    break;
	}
	i++;
    }
    
  
    return vect;
}


// Pubblica al task manager i task completati su "task_exec_topic"
void publishExecTask()
{
    task_assign::vect_task vect_msg;
}


// Pubblica ai robot i task rispettivamente assegnati
void publishRTAssign()
{
    task_assign::vect_task vect_msg;
}





int main(int argc, char **argv)
{
  
    // Initialize the node
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle node;
    
    obs_sub = node.subscribe("obstacles_topic", 20, &ObsCallback);
    assignment_sub = node.subscribe("assignment_topic", 20, &AssignCallback);
    status_rob_sub = node.subscribe("status_rob_topic", 20, &StatusCallback);
    
    exec_task_pub = node.advertise<task_assign::vect_task>("task_exec_topic", 10);   
    rob_ass_pub = node.advertise<task_assign::vect_robot>("rob_assign_topic", 10);
    rob_ini_pub = node.advertise<task_assign::vect_info>("rob_ini_topic", 10);
    rob_info_pub = node.advertise<task_assign::vect_info>("rob_info_topic", 10);
    rt_pub = node.advertise<task_assign::vect_task>("rt_topic", 10);
    
    sleep(1);

    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	ros::spinOnce();
	rate.sleep();
    }



    return 0;
}
