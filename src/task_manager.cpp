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


// gli elementi dei vettori potrebbero essere già dei type task_Assign::task così quando il nodo deve pubblicare il vettore 
// task_to_assign è già un vettore di elementi task, altrimenti le struct couple_task vanno copiate tutte nei type masg
vector<task_assign::task> new_task;          		//vettore in cui vengono messi i nuovi task in arrivo da users_node
vector<task_assign::task> executed_task;		//vettore in cui vengono messi i task eseguiti comunicati dal motion planner

vector<task_assign::task> task_to_assign;    		//T: vettore dei task da assegnare che cambia nel tempo di dim m(k)

int dim_n(0);


// Legge "task_arrival_topic" e mette i nuovi task nel vettore new_task
void NewCallback(const task_assign::vect_task::ConstPtr& msg)
{
    bool add_task(true);
    
    for(auto elem : msg->task_vect)
    {
	// vedo se elem sta già in new_task
	for(auto newel : new_task)
	{
	    if(newel.name1 == elem.name1 && newel.name2 == elem.name2)
		add_task = false;
	}
	
	if(add_task)
	    new_task.push_back(elem);
    }
}


// Legge "task_exec_topic" e mette i nuovi task nel vettore exec_task
void ExecCallback(const task_assign::vect_task::ConstPtr& msg)
{   
    bool add_task(true);
    
    for(auto elem : msg->task_vect)
    {
	// vedo se elem sta già in executed_task
	for(auto newel : executed_task)
	{
	    if(newel.name1 == elem.name1 && newel.name2 == elem.name2)
		add_task = false;
	}
	
	if(add_task)
	   executed_task.push_back(elem);
    }
}


// mette insieme executed_task(k), new_task(k) e task_to_assign(k-1) ed elabora un nuovo task_to_assign(k)
void taskManagement()
{
    bool add_task(true);
    
    for(auto elem : new_task)
    {
	// vedo se elem sta già in task_to_assign
	for(auto newel : task_to_assign)
	{
	    if(newel.name1 == elem.name1 && newel.name2 == elem.name2)
		add_task = false;
	}
	
	if(add_task)
	{  
	    task_to_assign.push_back(elem);
	}
    }
    
    bool erase_task(false);
    for(auto elem : executed_task)
    {
	// vedo se elem sta in task_to_assign
	int count(0);
	for(auto newel : task_to_assign)
	{
	    if(newel.name1 == elem.name1 && newel.name2 == elem.name2)
	    {
		erase_task = true;
		break;
	    }
	    count++;
	}
	
	if(erase_task)
	{  
	    task_to_assign.erase(task_to_assign.begin()+count);
	}
    }
  
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
    
    for(auto elem : assignment_msg.task_vect)
    {
	ROS_INFO_STREAM("The task_manager is publishing the task to assign: "<< elem.name << " whit the couple " << elem.name1 << " - " << elem.name2);
    }
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
