// Nodo che modella il task manager


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
ros::Subscriber new_task_sub;
ros::Subscriber exec_task_sub;
ros::Publisher task_ass_pub;


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


vector<struct couple_task> new_task;          	//vettore in cui vengono messi i nuovi task in arrivo da users_node
vector<struct couple_task> exec_task;		//vettore in cui vengono messi i task eseguiti comunicati dal motion planner
vector<struct couple_task> task_to_assign;    	//n(t): vettore dei task da assegnare che cambia nel tempo



// Legge "task_arrival_topic" e mette i nuovi task nel vettore new_task
void NewCallback(const task_assign::vect_task::ConstPtr& msg)
{

}


// Legge "task_exec_topic" e mette i nuovi task nel vettore exec_task
void ExecCallback(const task_assign::vect_task::ConstPtr& msg)
{

}


// Pubblica al central_node il vettore dei task da eseguire task_to_assign
void publishTaskToAssign()
{
    task_assign::vect_task vect_msg;
}





int main(int argc, char **argv)
{
  
    // Initialize the node
    ros::init(argc, argv, "task_manager");
    ros::NodeHandle node;
    
    new_task_sub = node.subscribe("task_arrival_topic", 20, &NewCallback);
    exec_task_sub = node.subscribe("task_exec_topic", 20, &ExecCallback);
    
    task_ass_pub = node.advertise<task_assign::vect_task>("task_assign_topic", 10);
    sleep(1);

    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	ros::spinOnce();
	rate.sleep();
    }



    return 0;
}