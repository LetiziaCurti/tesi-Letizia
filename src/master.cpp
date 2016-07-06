// Nodo che modella il master


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

ros::Publisher assignment_pub;
ros::Subscriber rob_ass_sub;
ros::Subscriber rob_info_sub;
ros::Subscriber task_ass_sub;


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


vector<struct couple_task> task_to_assign;    	//n(t): vettore dei task da assegnare che cambia nel tempo



// Legge su "task_assign_topic" il vettore dei task da eseguire n(t)
void TaskToAssCallback(const task_assign::vect_task::ConstPtr& msg)
{

}


// Legge su "rob_assign_topic" il vettore dei robot da assegnare m(t)
void RobToAssCallback(const task_assign::vect_task::ConstPtr& msg)
{

}


// Legge su "rob_info_topic" le info relative ai robot (tutti: già assegnati e da assegnare)
void RobInfoCallback(const task_assign::vect_task::ConstPtr& msg)
{

}


// Pubblica al motion planner gli assignments task-robot
void publishAssign()
{
    task_assign::vect_task vect_msg;
}





int main(int argc, char **argv)
{
  
    // Initialize the node
    ros::init(argc, argv, "master");
    ros::NodeHandle node;
    
    rob_ass_sub = node.subscribe("rob_assign_topic", 20, &RobToAssCallback);
    rob_info_sub = node.subscribe("rob_info_topic", 20, &RobInfoCallback);
    task_ass_sub = node.subscribe("task_assign_topic", 20, &TaskToAssCallback);
    
    assignment_pub = node.advertise<task_assign::vect_task>("assignment_topic", 10);
    
    sleep(1);

    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	ros::spinOnce();
	rate.sleep();
    }



    return 0;
}