// Nodo che modella il task manager
//  tiene traccia dei nuovi task (pubblicati dal nodo_utenti) e di quelli che vengono completati (pubblicati dal 
//  motion_planner), pubblica al central_node il vettore degli n(t) task da assegnare


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "task_assign/vect_task.h"
#include "task_assign/waypoint.h"


inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}



using namespace std;
ros::Subscriber new_task_sub;
ros::Subscriber exec_task_sub;
ros::Publisher new_task_pub;
ros::Publisher marker_pub;


// gli elementi dei vettori potrebbero essere già dei type task_Assign::task così quando il nodo deve pubblicare il vettore 
// task_to_assign è già un vettore di elementi task, altrimenti le struct couple_task vanno copiate tutte nei type masg
vector<task_assign::task> new_task;          		//vettore in cui vengono messi i nuovi task in arrivo da users_node
vector<task_assign::task> executed_task;		//vettore in cui vengono messi i task eseguiti comunicati dal motion planner

vector<task_assign::task> task_to_assign;    		//T: vettore dei task da assegnare che cambia nel tempo di dim m(k)

int dim_n(0);



// i task sono cilindri azzurri
void publishMarker(task_assign::waypoint p, int id_marker)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "robot_node";
    marker.id = id_marker;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = p.theta;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.5f;
    marker.color.g = 1.0f;
    marker.color.b = 0.8f;
    marker.color.a = 0.7;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
	break;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
}



void deleteMarker(task_assign::waypoint task_pose, int id_marker)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "task_node";
    marker.id = id_marker;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::DELETE;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = task_pose.x;
    marker.pose.position.y = task_pose.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = task_pose.theta;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.5f;
    marker.color.g = 1.0f;
    marker.color.b = 0.8f;
    marker.color.a = 0.7;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
// 	    return 0;
	break;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
}


    
// Function con cui viene data al robot la posizione passata in argomento, che viene poi inviata a tf
void broadcastPose(task_assign::waypoint posa, std::string name, int id_marker)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(posa.x, posa.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, posa.theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
    
    publishMarker(posa, id_marker);
}



// Legge "task_arrival_topic" e mette i nuovi task nel vettore new_task
void NewCallback(const task_assign::vect_task::ConstPtr& msg)
{
    bool add_task(true);
    task_assign::waypoint pos;
    
    for(auto elem : msg->task_vect)
    {
	// vedo se elem sta già in new_task
	for(auto newel : new_task)
	{
	    if(newel.id1 == elem.id1 && newel.id2 == elem.id2)
		add_task = false;
	}
	
	if(add_task)
	{
	    // vedo se elem sta già in executed_task
	    for(auto newel : executed_task)
	    {
		if(newel.id1 == elem.id1 && newel.id2 == elem.id2)
		    add_task = false;
	    }
	}
	
	if(add_task)
	{
	    new_task.push_back(elem);
	    
	    pos.x = elem.x1;
	    pos.y = elem.y1;
	    pos.theta = elem.theta1;
	    publishMarker(pos, elem.id1);
	    
	    pos.x = elem.x2;
	    pos.y = elem.y2;
	    pos.theta = elem.theta2;
	    publishMarker(pos, elem.id2);
	}
	
	add_task = true;
    }
}



// Legge "task_exec_topic" e mette i nuovi task nel vettore exec_task
void ExecCallback(const task_assign::vect_task::ConstPtr& msg)
{   
    bool add_task(true);
    task_assign::waypoint pos;
    
    for(auto elem : msg->task_vect)
    {
	// vedo se elem sta già in executed_task
	for(auto newel : executed_task)
	{
	    if(newel.id1 == elem.id1 && newel.id2 == elem.id2)
		add_task = false;
	}
	
	if(add_task)
	{
	   executed_task.push_back(elem);
	   
// 	   pos.x = elem.x1;
// 	   pos.y = elem.y1;
// 	   pos.theta = elem.theta1;
// 	   deleteMarker(pos, elem.id1);
	   
	   pos.x = elem.x2;
	   pos.y = elem.y2;
	   pos.theta = elem.theta2;
	   deleteMarker(pos, elem.id2);
	}
	
	add_task = true;
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
	    if(newel.id1 == elem.id1 && newel.id2 == elem.id2)
		add_task = false;
	}
	
	if(add_task)
	    task_to_assign.push_back(elem);
	
	add_task = true;
    }
    
    bool erase_task(false);
    for(auto elem : executed_task)
    {
	// vedo se elem sta in task_to_assign
	int count(0);
	for(auto newel : task_to_assign)
	{
	    if(newel.id1 == elem.id1 && newel.id2 == elem.id2)
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
	
	erase_task = false;
    }
  
    dim_n = task_to_assign.size();
}



// Pubblica al motion_planner il vettore dei nuovi task da eseguire task_to_assign
void publishTaskToAssign()
{
    task_assign::vect_task assignment_msg; 
    
    assignment_msg.task_vect = task_to_assign;

    // Wait for the publisher to connect to subscribers
    sleep(1.0);
    new_task_pub.publish(assignment_msg);
    
    for(auto elem : assignment_msg.task_vect)
    {
	ROS_INFO_STREAM("The task_manager is publishing the task to assign: "<< elem.name << " whit the couple " << elem.id1 << " - " << elem.id2);
    }
}









int main(int argc, char **argv)
{
  
    // Initialize the node
    ros::init(argc, argv, "task_manager");
    ros::NodeHandle node;
    
    new_task_sub = node.subscribe("task_arrival_topic", 20, &NewCallback);
    exec_task_sub = node.subscribe("task_exec_topic", 20, &ExecCallback);
    
    new_task_pub = node.advertise<task_assign::vect_task>("new_task_topic", 10);
    
    marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
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
