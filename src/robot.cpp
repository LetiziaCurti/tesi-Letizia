// Nodo che modella l'agente robot
// il robot comunica il suo stato (nome, status=false(not busy), posizione) al central 
// node (che lo impila nella coda in base al suo tempo di arrivo) scrivendo su "robot_arrival_topic", 
// finché non riceve un assignment dal central node (legge "assignment_topic")
// Dopodiché si muove per raggiungere con moveGoal il task.
// Infine pubblica su "assignment_topic" che è tornato libero

#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"  
#include <sys/stat.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "task_assign/vect_robot.h"
#include "task_assign/rt_vect.h"
#include "task_assign/assignment.h"
#include "task_assign/rech_vect.h"


#define DISTANCE_TOLERANCE 0.5
#define RECHARGE_DURATION 10

using namespace std;

struct quaternion
{
    double x;
    double y;
    double z;
    double w;
};

quaternion EulToQuat(double y, double z, double x) 
{
    quaternion Quat;
    // Assuming the angles are in radians.
    double c1 = cos(y);
    double s1 = sin(y);
    double c2 = cos(z);
    double s2 = sin(z);
    double c3 = cos(x);
    double s3 = sin(x);
    Quat.w = sqrt(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2.0;
    double w4 = (4.0 * Quat.w);
    Quat.x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4 ;
    Quat.y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4 ;
    Quat.z = (-s1 * s3 + c1 * s2 * c3 +s2) / w4 ;
    
    return Quat;
}

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}

double getDistance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

#define BATTERY_THR 10


class Robot
{
public:
  
    std::string robot_name;
    int id_marker;
    task_assign::waypoint turtlesim_pose;
    double b_level0, b_level;

    ros::Subscriber assignment_sub;
    ros::Subscriber recharge_sub;
    ros::Subscriber reassignment_sub;
    ros::Subscriber re_recharge_sub;
    
    ros::Publisher status_pub;
    ros::Publisher marker_pub;
    
    task_assign::waypoint taska_pose, taskb_pose;

    bool assignment = false;
    bool re_assignment = false;
    bool in_recharge = false;
    bool re_in_recharge = false;
    string task_name;
    int taska_id_marker, taskb_id_marker;
    int wait_a, wait_b;
    vector<task_assign::waypoint> path_a;
    vector<task_assign::waypoint> path_b;
    
    vector<task_assign::waypoint> new_path_a;
    vector<task_assign::waypoint> new_path_b;
    
    bool taska, taskb;
    float r, g, b;
    

    Robot(ros::NodeHandle& node, string name, int id, task_assign::waypoint pos, double b_l0, float red, float gr, float bl) 
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
	
	r = red;
	g = gr;
	b = bl;
	

	// Publish and subscribe to team status messages
	status_pub = node.advertise<task_assign::robot>("status_rob_topic", 10);	
	
	assignment_sub = node.subscribe("assignment_topic", 20, &Robot::AssignCallback,this);
	recharge_sub = node.subscribe("recharge_topic", 20, &Robot::RechargeCallback,this);
	
	reassignment_sub = node.subscribe("assignment_topic", 20, &Robot::ReAssignCallback,this);
	re_recharge_sub = node.subscribe("recharge_topic", 20, &Robot::Re_RechargeCallback,this);
	
	marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10, true);
    }
    
    
    
    vector<task_assign::waypoint> FindPath(task_assign::waypoint wp_now, vector<task_assign::waypoint> path_new)
    {
	vector<task_assign::waypoint> path_remain;
	bool is(false);
	int count(0);
	for(auto wp : path_new)
	{
	    count++;
	    if(wp.x == wp_now.x && wp.y == wp_now.y)
	    {
		is = true;
		break;
	    }
	}
	if(!is)
	    path_remain = path_new;
// 	else if(count==1)
// 		path_remain = path_new;
	else 
// 	  if(count>0)
	{
	    for(int i=count; i<path_new.size(); i++)
	    {
		path_remain.push_back(path_new[i]);
	    }
	}
			
	return path_remain;
    }
    
    
    
    // Il robot legge "assignment_topic" aspettando di ricevere un assignment
    // Pubblica il proprio stato su "robots_arrival_topic" finché non riceve un assignment, 
    // dopodiché parte per raggiungere il task seguendo i wp passati dal motion_planner
    void AssignCallback(const task_assign::assignment::ConstPtr& msg)
    {
	if(assignment) return;
	
	task_assign::waypoint wp;
	
	//check: deve essere arrivato qualcosa
	if(msg->assign_vect.size()>0)
	{	
	    for(auto elem : msg->assign_vect)
	    {
		
		// se l'assignment che leggo mi riguarda metto assignment a true così smetto di ascoltare 
		// altri messaggi finché non ho finito il task
		if(elem.r_name==robot_name && elem.t_name!=task_name)
		{
		    ROS_INFO_STREAM(robot_name << " is listening its assignment "<< elem.t_name <<" from motion_planner");
		    
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
		    
// 		    for(auto wp : path_a)
// 		    {
// 			ROS_INFO_STREAM("coordinate dei wp per taska: "<< wp.x <<" - " << wp.y);
// 		    }
				    
		    break;		  
		}  
	    }
	}
    }
    
    
    void ReAssignCallback(const task_assign::assignment::ConstPtr& msg)
    {
	if(re_assignment) return;
	
	//check: deve essere arrivato qualcosa
	if(msg->assign_vect.size()>0)
	{	
	    for(auto elem : msg->assign_vect)
	    {		
		// se l'assignment che leggo mi riguarda metto assignment a true così smetto di ascoltare 
		// altri messaggi finché non ho finito il task
		if(elem.r_name==robot_name && elem.t_name==task_name && elem.stop)
		{
		    if(elem.path_a.size() == path_a.size())
		    {
			int count(0);
			for(int i=0; i<elem.path_a.size(); i++)
			{
			    if(elem.path_a[i].x==path_a[i].x && elem.path_a[i].y==path_a[i].y)
				count++;
			    else
				break;			      
			}
			if(count!=elem.path_a.size())
			{
			    ROS_INFO_STREAM(robot_name << " is listening its REASSIGNMENT for "<< elem.t_name <<" from motion_planner");		    
			    re_assignment = true;
			    path_a = elem.path_a;
			}
		    }
		    else
		    {
			ROS_INFO_STREAM(robot_name << " is listening its REASSIGNMENT for "<< elem.t_name <<" from motion_planner");		    
			re_assignment = true;
			path_a = elem.path_a;
		    }
		    
		    
		    if(elem.path_b.size() == path_b.size())
		    {
			int count(0);
			for(int i=0; i<elem.path_b.size(); i++)
			{
			    if(elem.path_b[i].x==path_b[i].x && elem.path_b[i].y==path_b[i].y)
				count++;
			    else
				break;			      
			}
			if(count!=elem.path_b.size())
			{
			    ROS_INFO_STREAM(robot_name << " is listening its REASSIGNMENT for "<< elem.t_name <<" from motion_planner");		    
			    re_assignment = true;
			    path_b = elem.path_b;
			}
		    }
		    else
		    {
			ROS_INFO_STREAM(robot_name << " is listening its REASSIGNMENT for "<< elem.t_name <<" from motion_planner");		    
			re_assignment = true;
			path_b = elem.path_b;
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
		// se devo andare in ricarica metto in_recharge a true così smetto di ascoltare
		// altri messaggi finché non ho finito di ricaricarmi
		if(elem.r_name==robot_name && elem.t_name!=task_name)
		{
		    ROS_INFO_STREAM(robot_name << " is listening its recharge point "<< elem.t_name <<" from motion_planner");
		    
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
		    
		    for(auto wp : path_a)
		    {
			ROS_INFO_STREAM("coordinate dei wp per il rech. point: "<< wp.x <<" - " << wp.y);
		    }
		    
		    break;
		}
	    }   
	}
    }
    
    
    
    void Re_RechargeCallback(const task_assign::assignment::ConstPtr& msg)
    {
	if(re_in_recharge) return;
	
	//check: deve essere arrivato qualcosa
	if(msg->assign_vect.size()>0)
	{	
	    for(auto elem : msg->assign_vect)
	    {
		
		// se l'assignment che leggo mi riguarda metto assignment a true così smetto di ascoltare 
		// altri messaggi finché non ho finito il task
		if(elem.r_name==robot_name && elem.t_name==task_name && elem.stop)
		{
		    if(elem.path_a.size() == path_a.size())
		    {
			int count(0);
			for(int i=0; i<elem.path_a.size(); i++)
			{
			    if(elem.path_a[i].x==path_a[i].x && elem.path_a[i].y==path_a[i].y)
				count++;
			    else
				break;			      
			}
			if(count!=elem.path_a.size())
			{
			    ROS_INFO_STREAM(robot_name << " is listening its REASSIGNMENT for "<< elem.t_name <<" from motion_planner");		    
			    re_in_recharge = true;
			    path_a = elem.path_a;
			}
		    }
		    else
		    {
			ROS_INFO_STREAM(robot_name << " is listening its REASSIGNMENT for "<< elem.t_name <<" from motion_planner");		    
			re_in_recharge = true;
			path_a = elem.path_a;
		    }
		    
		    break;
		}   
	    }
	}
    }
    
    
    
    // Il robot pubblica il suo stato su "status_rob_topic"
    void publishStatus() 
    {
	task_assign::robot status_msg; 
	broadcastPose(turtlesim_pose,robot_name);

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
	    status_msg.x = floor(turtlesim_pose.x+0.5);
	    status_msg.y = floor(turtlesim_pose.y+0.5);
	    status_msg.theta = floor(turtlesim_pose.theta+0.5);
	}

	// Wait for the publisher to connect to subscribers
	sleep(2);
	status_pub.publish(status_msg);
	
	ROS_INFO_STREAM("Robot "<< robot_name <<" is publishing its status "<< BoolToString(status_msg.status));
	ROS_INFO_STREAM("Robot "<< robot_name <<" is publishing its position");
	ROS_INFO_STREAM("x: " << status_msg.x << " y: " << status_msg.y);
	ROS_INFO_STREAM("Robot "<< robot_name <<" is publishing its battery level " << status_msg.b_level);
    }
    
        // Il robot pubblica il suo stato su "status_rob_topic"
    void publishWp() 
    {
	task_assign::robot status_msg; 
	broadcastPose(turtlesim_pose,robot_name);

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
	    status_msg.x = floor(turtlesim_pose.x+0.5);
	    status_msg.y = floor(turtlesim_pose.y+0.5);
	    status_msg.theta = floor(turtlesim_pose.theta+0.5);
	}

	status_pub.publish(status_msg);
	
	ROS_INFO_STREAM("Robot "<< robot_name <<" is publishing its WP");
	ROS_INFO_STREAM("x: " << status_msg.x << " y: " << status_msg.y);
    }
    
    
    
    // Function for bringing the robot in the position of the task to accomplish and then in the position of
    // the exit
    void moveToWP(vector <task_assign::waypoint> wps, double distance_tolerance)
    {
	double vel_x;
	double vel_z;
	double time = 0.4;
	ros::Rate rate(10);
	
	for(auto goal_pose : wps)
	{
	    if(ros::ok() && !re_assignment && !re_in_recharge)
	    {
		
		ROS_INFO_STREAM("ROBOT "<< robot_name <<" IS MOVING TO " << task_name);
		do{
		      broadcastPose(turtlesim_pose,robot_name);
		      
		      vel_x = 0.5*getDistance(turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y);
		      vel_z = 4*sin((atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta));
		      
		      turtlesim_pose.x = (vel_x*cos(turtlesim_pose.theta))*time + turtlesim_pose.x;
		      turtlesim_pose.y = (vel_x*sin(turtlesim_pose.theta))*time + turtlesim_pose.y;
		      turtlesim_pose.theta = sin(vel_z*time) + turtlesim_pose.theta;	
		      
		      b_level-=0.01;
		      if(b_level<BATTERY_THR)
			  return;
		      
		      ros::spinOnce();		      
		      rate.sleep();
		      
		}while(getDistance(turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y)>distance_tolerance && ros::ok() && !re_assignment && !re_in_recharge);
		
		publishWp();
	    }
	    if(re_assignment || re_in_recharge)
		break;

	}
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
// 	marker.type = visualization_msgs::Marker::SPHERE;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = "package://task_assign/config/Taxi.stl";

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = p.x;
	marker.pose.position.y = p.y;
	marker.pose.position.z = 0;
	quaternion Quat;
	Quat = EulToQuat(0.0,p.theta+1.57,1.57);
	marker.pose.orientation.x = Quat.x;
	marker.pose.orientation.y = Quat.y;
	marker.pose.orientation.z = Quat.z;
	marker.pose.orientation.w = Quat.w;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.05;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 1;

	marker.lifetime = ros::DURATION_MAX;

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
    float r = atof(argv[7]);
    float g = atof(argv[8]);
    float b = atof(argv[9]);
    Robot robot(node, name, id, pose, b_level0, r, g, b);

    sleep(1); 

    vector<task_assign::waypoint> new_path;
    task_assign::waypoint wp;
    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	robot.new_path_a.clear();
	robot.new_path_b.clear();
	
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
	    if(robot.re_assignment && robot.new_path_a.size()>0)
	    {
		wp.x = floor(robot.turtlesim_pose.x+0.5);
		wp.y = floor(robot.turtlesim_pose.y+0.5);
		wp.theta = robot.turtlesim_pose.theta;
		new_path = robot.FindPath(wp, robot.new_path_a);
		robot.re_assignment = false;
		robot.moveToWP(new_path, DISTANCE_TOLERANCE);
// 		robot.moveToWP(robot.path_a, DISTANCE_TOLERANCE);
	    }
	    sleep(robot.wait_a);
	    robot.b_level -= robot.wait_a;
	    if(robot.b_level<BATTERY_THR)
		return 0;
	    robot.taska = true;
// 	    robot.publishStatus(); 

	    robot.publishStatus();
	    if(robot.new_path_b.size()>0)
		robot.moveToWP(robot.new_path_b, DISTANCE_TOLERANCE);
	    else
		robot.moveToWP(robot.path_b, DISTANCE_TOLERANCE);
	    if(robot.re_assignment)
	    {
		wp.x = floor(robot.turtlesim_pose.x+0.5);
		wp.y = floor(robot.turtlesim_pose.y+0.5);
		wp.theta = robot.turtlesim_pose.theta;
		new_path = robot.FindPath(wp, robot.new_path_b);
		robot.re_assignment = false;		
		robot.moveToWP(new_path, DISTANCE_TOLERANCE);
// 		robot.moveToWP(robot.path_b, DISTANCE_TOLERANCE);		
	    }
	    sleep(robot.wait_b);
	    ROS_INFO_STREAM("ROBOT "<< robot.robot_name <<" HA COMPLETATO " << robot.task_name);
	    
	    robot.b_level -= robot.wait_b;
	    if(robot.b_level<BATTERY_THR)
		return 0;
	    robot.taskb = true;
	    robot.assignment = false;	 
	    robot.re_assignment = false;
	    robot.publishStatus();  
	}
	
	else if(robot.in_recharge && ros::ok())
	{  
	    // il robot va a ricaricarsi
	    ROS_INFO_STREAM("ROBOT "<< robot.robot_name <<" IS GOING TO RECHARGE IN  " << robot.task_name);

	    robot.publishStatus(); 
	    robot.moveToWP(robot.path_a, DISTANCE_TOLERANCE);	 
	    if(robot.re_in_recharge && robot.new_path_a.size()>0)
	    {
		wp.x = floor(robot.turtlesim_pose.x+0.5);
		wp.y = floor(robot.turtlesim_pose.y+0.5);
		wp.theta = robot.turtlesim_pose.theta;
		new_path = robot.FindPath(wp, robot.path_a);
		robot.re_in_recharge = false;
		robot.moveToWP(new_path, DISTANCE_TOLERANCE);
// 		robot.moveToWP(robot.path_a, DISTANCE_TOLERANCE);
	    }
	    sleep(RECHARGE_DURATION);
	    ROS_INFO_STREAM("ROBOT "<< robot.robot_name <<" SI E' RICARICATO IN " << robot.task_name);
	    
	    robot.b_level = b_level0;
	    robot.taska = true;
	    robot.in_recharge = false;
	    robot.re_in_recharge = false;
	    robot.publishStatus(); 
	}
	
	robot.taska = false;
	robot.taskb = false;
	
	robot.broadcastPose(robot.turtlesim_pose, name);
	ros::spinOnce(); 
	rate.sleep();
    }
 
    return 0;
}
