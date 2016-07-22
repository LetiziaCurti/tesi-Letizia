// Nodo che modella il task manager
//  tiene traccia dei nuovi task (pubblicati dal nodo_utenti) e di quelli che vengono completati (pubblicati dal 
//  motion_planner), pubblica al central_node il vettore degli n(t) task da assegnare


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"
#include "task_assign/AssignMsg.h"


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

// gli elementi dei vettori potrebbero essere già dei type task_Assign::task così quando il nodo deve pubblicare il vettore 
// task_to_assign è già un vettore di elementi task, altrimenti le struct couple_task vanno copiate tutte nei type masg
vector<struct couple_task> new_task;          		//vettore in cui vengono messi i nuovi task in arrivo da users_node
vector<struct couple_task> executed_task;		//vettore in cui vengono messi i task eseguiti comunicati dal motion planner

vector<task_assign::task> task_to_assign;    		//n(t): vettore dei task da assegnare che cambia nel tempo

int dim_n(0);


// Legge "task_arrival_topic" e mette i nuovi task nel vettore new_task
void NewCallback(const task_assign::vect_task::ConstPtr& msg)
{
    bool add_task(true);
    struct couple_task task;
    
    for(auto elem : msg->task_vect)
    {
	// vedo se elem sta già in new_task
	for(auto newel : new_task)
	{
	    if(newel.name1 == elem.name1 && newel.name2 == elem.name2)
	      add_task = false;
	}
	
	if(add_task)
	{
	   task.ar_time = elem.ar_time;
	   task.id1 = elem.id1;
	   task.name1 = elem.name1;
	   task.status1 = elem.status1;
	   task.wait1 = elem.wait1;
	   task.x1 = elem.x1;
	   task.y1 = elem.y1;
	   task.theta1 = elem.theta1;
	   
	   task.id2 = elem.id2;
	   task.name2 = elem.name2;
	   task.status2 = elem.status2;
	   task.wait2 = elem.wait2;
	   task.x2 = elem.x2;
	   task.y2 = elem.y2;
	   task.theta2 = elem.theta2;
	   
	   new_task.push_back(task);
	}
    }
}


// Legge "task_exec_topic" e mette i nuovi task nel vettore exec_task
void ExecCallback(const task_assign::vect_task::ConstPtr& msg)
{   
    bool add_task(true);
    struct couple_task task;
    
    for(auto elem : msg->task_vect)
    {
	// vedo se elem sta già in executed_task
	for(auto newel : executed_task)
	{
	    if(newel.name1 == elem.name1 && newel.name2 == elem.name2)
	      add_task = false;
	}
	
	if(add_task)
	{
	   task.ar_time = elem.ar_time;
	   task.id1 = elem.id1;
	   task.name1 = elem.name1;
	   task.status1 = elem.status1;
	   task.wait1 = elem.wait1;
	   task.x1 = elem.x1;
	   task.y1 = elem.y1;
	   task.theta1 = elem.theta1;
	   
	   task.id2 = elem.id2;
	   task.name2 = elem.name2;
	   task.status2 = elem.status2;
	   task.wait2 = elem.wait2;
	   task.x2 = elem.x2;
	   task.y2 = elem.y2;
	   task.theta2 = elem.theta2;
	   
	   executed_task.push_back(task);
	}
    }
}


// mette insieme executed_task(k), new_task(k) e task_to_assign(k-1) ed elabora un nuovo task_to_assign(k)
void taskManagement()
{
    dim_n = task_to_assign.size();
}


// Pubblica al master il vettore dei task da eseguire task_to_assign
void publishTaskToAssign()
{
    task_assign::vect_task assignment_msg; 
    
    assignment_msg.task_vect = task_to_assign;

    // Wait for the publisher to connect to subscribers
    sleep(1.0);
    task_ass_pub.publish(assignment_msg);
    
//     for(auto elem : task_to_assign)
//     {
// 	ROS_INFO_STREAM("Task Manager is publishing the assignment: "<< elem.task_id << " - " << elem.rob_id);
//     }
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
	taskManagement();
	
	if(dim_n > 0)
	    publishTaskToAssign();
	
	ros::spinOnce();
	rate.sleep();
    }



    return 0;
}
