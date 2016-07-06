// Nodo che modella il motion planner


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"


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
ros::Publisher rob_info_pub;
ros::Publisher rt_pub;


struct couple_task
{
    double ar_time;
    string name1;
    int id1;
    bool status1;
    double wait1;
    float x1;
    float y1;
    float theta1;
    
    string name2;
    int id2;
    bool status2;
    double wait2;
    float x2;
    float y2;
    float theta2;
};





// Legge "obstacles_topic" 
void ObsCallback(const task_assign::vect_task::ConstPtr& msg)
{

}


// Legge "assignment_topic" 
void AssignCallback(const task_assign::vect_task::ConstPtr& msg)
{

}


// Legge "status_rob_topic" 
void StatusCallback(const task_assign::vect_task::ConstPtr& msg)
{

}


// Pubblica al master su "rob_assign_topic il vettore dei robot da assegnare
void publishRobotToAssign()
{
    task_assign::vect_task vect_msg;
}


// Pubblica al master su "rob_info_topic" le info relative ai robot (tutti: già assegnati e da assegnare)
void publishRobotInfo()
{
    task_assign::vect_task vect_msg;
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
    rob_ass_pub = node.advertise<task_assign::vect_task>("rob_assign_topic", 10);
    rob_info_pub = node.advertise<task_assign::vect_task>("rob_info_topic", 10);
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