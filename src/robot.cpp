// Nodo che modella l'agente robot
// il robot comunica il suo stato (nome, status=false(not busy), posizione) al central 
// node (che lo impila nella coda in base al suo tempo di arrivo) scrivendo su "robot_arrival_topic", 
// finché non riceve un assignment dal central node (legge "assignment_topic")
// Dopodiché si muove per raggiungere con moveGoal il task.
// Infine pubblica su "assignment_topic" che è tornato libero

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"  
#include "task_assign/IniStatus.h" 
#include "task_assign/OneAssign.h"
#include "task_assign/AssignMsg.h"
#include <sys/stat.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "task_assign/vect_robot.h"
#include "task_assign/rt_vect.h"
#include "task_assign/assignment.h"
#include "task_assign/rech_vect.h"


#define DISTANCE_TOLERANCE 0.01
#define RECHARGE_DURATION 10

using namespace std;

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}

double getDistance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}


class Robot
{
public:
  
    std::string robot_name;
    int id_marker;
    task_assign::waypoint turtlesim_pose;
    double b_level0, b_level;

    ros::Subscriber assignment_sub;
    ros::Subscriber recharge_sub;
    ros::Publisher status_pub;
    ros::Publisher marker_pub;
    
    task_assign::waypoint taska_pose, taskb_pose;

    bool assignment = false;
    bool in_recharge = false;
    string task_name;
    int taska_id_marker, taskb_id_marker;
    double wait_a, wait_b;
    vector<task_assign::waypoint> path_a;
    vector<task_assign::waypoint> path_b;
    
    bool taska, taskb;
    

    Robot(ros::NodeHandle& node, string name, int id, task_assign::waypoint pos, double b_l0) 
    {
	robot_name = name;
	id_marker = id;
	
	b_level0 = b_l0;
	b_level = b_l0;
	
	taska = false;
	taskb = false;
	
	turtlesim_pose.x = pos.x;
	turtlesim_pose.y = pos.y;
	turtlesim_pose.theta = pos.theta;
	

	// Publish and subscribe to team status messages
	status_pub = node.advertise<task_assign::robot>("status_rob_topic", 10);	
	
	assignment_sub = node.subscribe("assignment_topic", 20, &Robot::AssignCallback,this);
	recharge_sub = node.subscribe("recharge_topic", 20, &Robot::RechargeCallback,this);
	
	marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }
    
    
    
    // Il robot legge "assignment_topic" aspettando di ricevere un assignment
    // Pubblica il proprio stato su "robots_arrival_topic" finché non riceve un assignment, 
    // dopodiché parte per raggiungere il task seguendo i wp passati dal motion_planner
    void AssignCallback(const task_assign::assignment::ConstPtr& msg)
    {
	task_assign::waypoint wp;
// 	ROS_INFO_STREAM(robot_name << " I AM LISTENING THE ASSIGNMENT FROM MOTION PLANNER");

	if(assignment) return;
	
	for(auto elem : msg->assign_vect)
	{
	    //check: deve essere arrivato qualcosa
	    if(elem.t_name!="" && elem.r_name!="")
	    {	    
		// se l'assignment che leggo mi riguarda metto assignment a true così smetto di ascoltare 
		// altri messaggi finché non ho finito il task
		if(elem.r_name==robot_name)
		{
		    ROS_INFO_STREAM(robot_name << " is listening its assignment from motion_planner");
		    
		    assignment = true;
		    task_name = elem.t_name;
		    taska_id_marker = elem.id_a;
		    taskb_id_marker = elem.id_b;
		    
		    // è l'ultimo wp di path_a
		    path_a = elem.path_a;
		    wp = path_a.back();
		    taska_pose.x = wp.x;
		    taska_pose.y = wp.y;
		    taska_pose.theta = wp.theta;
		    wait_a = wp.wait;
		    
		    // è l'ultimo wp di path_b
		    path_b = elem.path_b;
		    wp = path_b.back();
		    taskb_pose.x = wp.x;
		    taskb_pose.y = wp.y;
		    taskb_pose.theta = wp.theta;
		    wait_b = wp.wait;
		    
		    for(auto wp : path_a)
		    {
			ROS_INFO_STREAM("coordinate dei wp per taska: "<< wp.x <<" - " << wp.y);
		    }
		    		    
		    break;		  
		}
	    }   
	}
    }
    
    
    
    void RechargeCallback(const task_assign::assignment::ConstPtr& msg)
    {
	task_assign::waypoint wp;
	
	if(in_recharge) return;
	
	for(auto elem : msg->assign_vect)
	{
	    //check: deve essere arrivato qualcosa
	    if(elem.t_name!="" && elem.r_name!="")
	    {
		ROS_INFO_STREAM(robot_name << " is listening its recharge point from motion_planner");	    
		// se devo andare in ricarica metto in_recharge a true così smetto di ascoltare
		// altri messaggi finché non ho finito di ricaricarmi
		if(elem.r_name==robot_name)
		{
		    in_recharge = true;
		    task_name = elem.t_name;
		    taska_id_marker = elem.id_a;
		    
		    // i punti di ricarica sono task che hanno solo la prima parte (taska)
		    // è l'ultimo wp di path_a
		    path_a = elem.path_a;
		    wp = path_a.back();
		    taska_pose.x = wp.x;
		    taska_pose.y = wp.y;
		    taska_pose.theta = wp.theta;
		    wait_a = wp.wait;
		    
		    break;
		}
	    }   
	}
    }
    
    
    
    // Function for bringing the robot in the position of the task to accomplish and then in the position of
    // the exit
    void moveToWP(vector <task_assign::waypoint> wps, double distance_tolerance)
    {
	double vel_x;
	double vel_z;
	double time = 0.4;
	
	for(auto goal_pose : wps)
	{
	    ros::Rate rate(10);
	    do{
		  publishMarker(turtlesim_pose);
		  broadcastPose(turtlesim_pose,robot_name);
		  
		  vel_x = 0.5*getDistance(turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y);
		  vel_z = 4*sin((atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta));
		  
		  turtlesim_pose.x = (vel_x*cos(turtlesim_pose.theta))*time + turtlesim_pose.x;
		  turtlesim_pose.y = (vel_x*sin(turtlesim_pose.theta))*time + turtlesim_pose.y;
		  turtlesim_pose.theta = sin(vel_z*time) + turtlesim_pose.theta;	
		  
		  b_level-=0.01;

		  ros::spinOnce();
		  rate.sleep(); 
	    }while(getDistance(turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y)>distance_tolerance && ros::ok());
	}
	
// 	deleteMarker(goal_pose, task_id_marker);

    }
    
    
    
    // Il robot pubblica il suo stato su "status_rob_topic"
    void publishStatus() 
    {
	task_assign::robot status_msg; 

	status_msg.header.stamp = ros::Time::now();
	status_msg.name = robot_name;
	status_msg.id = id_marker;
	status_msg.status = true;
	status_msg.b_level0 = b_level0;
	status_msg.b_level = b_level;
	status_msg.taska = taska;
	status_msg.taskb = taskb;
	
	if(turtlesim_pose.x!=-1 && turtlesim_pose.y!=-1 && turtlesim_pose.theta!=200)
	{
	    status_msg.x = turtlesim_pose.x;
	    status_msg.y = turtlesim_pose.y;
	    status_msg.theta = turtlesim_pose.theta;
	}

	// Wait for the publisher to connect to subscribers
	sleep(2);
	status_pub.publish(status_msg);
	
	ROS_INFO_STREAM("Robot "<< robot_name <<" is publishing its status "<< BoolToString(status_msg.status));
    }

    
    
    void publishMarker(task_assign::waypoint p)
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
	marker.type = visualization_msgs::Marker::SPHERE;

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
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
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
 
 

    void deleteMarker(task_assign::waypoint task_pose, int t_id)
    {
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "task_node";
	marker.id = t_id;

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
	marker.color.r = 1.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
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
    void broadcastPose(task_assign::waypoint posa, std::string name)
    {
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(posa.x, posa.y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, posa.theta);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
	
	publishMarker(posa);
    }


    
    

};


int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "robot_node");
    
    ros::NodeHandle node;
    string name = std::string(argv[1]);
    int id = atoi(argv[2]);
    
    task_assign::waypoint pose;
    pose.x = atof(argv[3]);
    pose.y = atof(argv[4]);
    pose.theta = atof(argv[5]);
    double b_level0 = atof(argv[6]);
    Robot robot(node, name, id, pose, b_level0);

    sleep(1); 



    ros::Rate rate(10);
    while (ros::ok()) 
    {
	while(!robot.assignment && !robot.in_recharge && ros::ok())
	{
	    robot.broadcastPose(robot.turtlesim_pose, name);
	    robot.publishStatus();  
	    ros::spinOnce();
	    rate.sleep();
	}
	
	if(robot.assignment && ros::ok())
	{  
	    // il robot si muove verso il task
	    ROS_INFO_STREAM("ROBOT "<< robot.robot_name <<" IS MOVING TO " << robot.task_name);
	    
	    robot.publishStatus(); 
	    robot.moveToWP(robot.path_a, DISTANCE_TOLERANCE);
	    sleep(robot.wait_a);
	    robot.b_level -= robot.wait_a*0.1;
	    robot.taska = true;
	    robot.publishStatus(); 
// 	    robot.deleteMarker(robot.path_a.back(), robot.taska_id_marker);

	    robot.publishStatus(); 
	    robot.moveToWP(robot.path_b, DISTANCE_TOLERANCE);
	    sleep(robot.wait_b);
	    robot.b_level -= robot.wait_b*0.1;
	    robot.taskb = true;
	    robot.publishStatus(); 
// 	    robot.deleteMarker(robot.taskb_pose, robot.taskb_id_marker);
 
	    robot.assignment = false;
	}
	
	else if(robot.in_recharge && ros::ok())
	{  
	    // il robot va a ricaricarsi
	    ROS_INFO_STREAM("ROBOT "<< robot.robot_name <<" IS GOING TO RECHARGE IN  " << robot.task_name);

	    robot.publishStatus(); 
	    robot.moveToWP(robot.path_a, DISTANCE_TOLERANCE);
	    
	    sleep(RECHARGE_DURATION);
	    robot.b_level = b_level0;
	    robot.publishStatus(); 
	    
	    robot.in_recharge = false;
	}
	
	robot.broadcastPose(robot.turtlesim_pose, name);
	ros::spinOnce(); 
	rate.sleep();
    }
 
    return 0;
}
