// Nodo che modella gli ostacoli e gli imprevisti


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"


using namespace std;
ros::Publisher obs_pub;



void publishObstacles()
{
    task_assign::vect_task vect_msg;
    
    
    // Wait for the publisher to connect to subscribers
    sleep(1.0);
    obs_pub.publish(vect_msg);
}


inline const char * const BoolToString(bool b)
{
    return b ? "true" : "false";
}




int main(int argc, char **argv)
{
  
    // Initialize the node
    ros::init(argc, argv, "obstacle_node");
    ros::NodeHandle node;
    
    obs_pub = node.advertise<task_assign::vect_task>("obstacles_topic", 10);
    sleep(1);
    
    
    
    // Leggi ostacoli e imprevisti da un file yaml e pubblicali con publishObstacles()
    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	publishObstacles();
	ros::spinOnce();
	rate.sleep();
    }



    return 0;
}