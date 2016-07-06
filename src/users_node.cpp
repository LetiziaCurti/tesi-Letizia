// Nodo che modella le richieste degli utenti


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"


using namespace std;
ros::Publisher new_task_pub;


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



void publishVectTask(vector<struct couple_task> new_task_vect)
{
    task_assign::vect_task vect_msg;
    
    int count(0);
    for(auto task : new_task_vect)
    {	
	vect_msg.task_vect[count].ar_time = task.ar_time;
	
	vect_msg.task_vect[count].name1 = task.name1;
	vect_msg.task_vect[count].id1 = task.id1;
	vect_msg.task_vect[count].status1 = task.status1;
	vect_msg.task_vect[count].wait1 = task.wait1;
	vect_msg.task_vect[count].x1 = task.x1;
	vect_msg.task_vect[count].y1 = task.y1;
	vect_msg.task_vect[count].theta1 = task.theta1;
	
	vect_msg.task_vect[count].name2 = task.name2;
	vect_msg.task_vect[count].id2 = task.id2;
	vect_msg.task_vect[count].status2 = task.status2;
	vect_msg.task_vect[count].wait2 = task.wait2;
	vect_msg.task_vect[count].x2 = task.x2;
	vect_msg.task_vect[count].y2 = task.y2;
	vect_msg.task_vect[count].theta2 = task.theta2;

	count++;
    }
    
    // Wait for the publisher to connect to subscribers
    sleep(1.0);
    new_task_pub.publish(vect_msg);
}


inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}




int main(int argc, char **argv)
{
  
    // Initialize the node
    ros::init(argc, argv, "users_node");
    ros::NodeHandle node;
    
    new_task_pub = node.advertise<task_assign::vect_task>("task_arrival_topic", 10);
    sleep(1);
    
    
    
    // Leggi tutte le info da un file yaml e mettile al posto di at e new_task_vect
    double at;
    vector<struct couple_task> new_task_vect;
    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	sleep(at);
	publishVectTask(new_task_vect);
	ros::spinOnce();
	rate.sleep();
    }



    return 0;
}
