// Nodo che modella l'agente robot
// il robot comunica il suo stato (nome, status=false(not busy), posizione) al central 
// node (che lo impila nella coda in base al suo tempo di arrivo) scrivendo su "robot_arrival_topic", 
// finché non riceve un assignment dal central node (legge "assignment_topic")
// Dopodiché pubblica su "cmd_vel topic" per raggiungere con moveGoal il task (e successivamente l'uscita)
// (Raggiunta l'uscita) poi pubblica su "assignment_topic" che lui è libero

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

#define DISTANCE_TOLERANCE 0.01

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

    ros::Subscriber assignment_sub;
    ros::Publisher assignment_pub;
    ros::Publisher status_pub;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Publisher marker_pub;

    ros::Time t_arrive;
    turtlesim::Pose turtlesim_pose;
    turtlesim::Pose task_pose;
    turtlesim::Pose uscita;

    bool assignment = false;
    string task_name;
    int task_id_marker;
    

    Robot(ros::NodeHandle& node, string name, int id) 
    {
	robot_name = name;
	id_marker = id;
	
	turtlesim_pose.x=-1;
	turtlesim_pose.y=-1;
	turtlesim_pose.theta=200;
	
	uscita.x = 10;
	uscita.y = 1;
	uscita.theta = 0;
	
	
	// A publisher for the movement data
	pub = node.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 10);
	// A listener for pose
	sub = node.subscribe(robot_name + "/pose", 10, &Robot::poseCallback,this);

	// Publish and subscribe to team status messages
	status_pub = node.advertise<task_assign::IniStatus>("robot_arrival_topic", 10);
	
	assignment_pub = node.advertise<task_assign::OneAssign>("free_assign_topic", 10);
	assignment_sub = node.subscribe("assignment_topic", 20, &Robot::AssignCallback,this);
	
	marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }


    // Callback con cui il robot legge la sua posizione dal /sim node
    void poseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
	//ROS_INFO("x: %.2f, y: %.2f, theta: %.2f", msg->x, msg->y, msg->theta);
	turtlesim_pose.x = msg -> x;
	turtlesim_pose.y = msg -> y;
	turtlesim_pose.theta = msg -> theta;
	
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_name));
	
	publishMarker(turtlesim_pose);
    }
    
    
    void publishMarker(turtlesim::Pose turtlesim_pose)
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
	marker.pose.position.x = turtlesim_pose.x;
	marker.pose.position.y = turtlesim_pose.y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = turtlesim_pose.theta;
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



    // Il robot pubblica il suo stato su "robot_arrival_topic"
    void publishIniStatus() 
    {
	task_assign::IniStatus status_msg; 

	status_msg.header.stamp = ros::Time::now();
	status_msg.t = ros::Time::now();
	status_msg.robot_id = robot_name;
	status_msg.is_ready = true;
	status_msg.status = false; //I am available (not busy)
	status_msg.type = "robot";
	
	if(turtlesim_pose.x!=-1 && turtlesim_pose.y!=-1 && turtlesim_pose.theta!=200)
	{
	    status_msg.x = turtlesim_pose.x;
	    status_msg.y = turtlesim_pose.y;
	    status_msg.theta = turtlesim_pose.theta;
	}

	// Wait for the publisher to connect to subscribers
	sleep(1);
	status_pub.publish(status_msg);
	
	ROS_INFO_STREAM("Robot "<< robot_name <<" is publishing its initial status "<< BoolToString(status_msg.status));
    }
    
    
    void deleteMarker(turtlesim::Pose task_pose, int t_id)
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
	marker.type = visualization_msgs::Marker::CUBE;

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



    // Function for bringing the robot in the position of the task to accomplish and then in the position of
    // the exit
    void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance)
    {
      
	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(10);
	do{
    // 	  Proportional Controller
	    
    // 	  linear velocity in the x-axis
	      vel_msg.linear.x = 0.5*getDistance(turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y);
	      vel_msg.linear.y = 0;
	      vel_msg.linear.z = 0;
	    
    // 	  angular velocity in the z-axis
	      vel_msg.angular.x = 0;
	      vel_msg.angular.y = 0;
	      vel_msg.angular.z = 4*sin((atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta));
	    
	      pub.publish(vel_msg);
	      ros::spinOnce();
	      loop_rate.sleep();  
	}while(getDistance(turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y)>distance_tolerance && ros::ok());
	
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	pub.publish(vel_msg);
    }



    // Il robot pubblica su "assignment_topic" un messaggio contenente lo stato del task che ha eseguito
    // e il suo stato (nel campo msg.robot_assign), rimettendo a false gli status suo e del task eseguito
    void publishFreeStatus() 
    {
	task_assign::OneAssign status_msg; 

	status_msg.task_id = task_name;
	status_msg.t_ready = true;
	status_msg.t_status = true; //tengo lo stato del task a true (anche se dovrebbe essere comunicato false al task per fargli capire che è stato terminato)
	status_msg.task_x = task_pose.x;
	status_msg.task_y = task_pose.y;
	status_msg.task_theta = task_pose.theta;
	
	//pubblico robot_assign
	status_msg.rob_id = robot_name;
	status_msg.r_status = false;  //rimetto il mio stato su false


	// Wait for the publisher to connect to subscribers
// 	sleep(1.0);
	assignment_pub.publish(status_msg);
	
	ROS_INFO_STREAM(robot_name <<" has finished the assignment. So "<< task_name << " is executed.");
    }




    // Il robot legge "assignment_topic" aspettando di ricevere un assignment
    // Pubblica il proprio stato di ready su "robots_arrival_topic" finché non riceve un assignment, 
    // dopodiché smette di pubblicare il suo stato e memorizza la posizione del task da raggiungere
    // in task_pose
    void AssignCallback(const task_assign::AssignMsg::ConstPtr& status_msg)
    {
	if(assignment) return;
	
	for(auto elem : status_msg->assign_vect)
	{
	    //check: deve essere arrivato qualcosa
	    if(elem.t_ready && elem.task_x!=0 && elem.task_y!=0 && elem.task_theta!=0)
	    {
// 		ROS_INFO_STREAM(robot_name << " is listening " << status_msg->robot_id << " with robot assigned " << status_msg->robot_assign.id);
	    
		// se il task che è arrivato ha come robot assegnato me, metto assignment a true così
		// smetto di pubblicare il mio stato
		if(elem.rob_id==robot_name && elem.r_status==true)
		{
		    assignment = true;
		    task_name = elem.task_id;
		    task_id_marker = elem.t_id_marker;
		    
		    task_pose.x = elem.task_x;
		    task_pose.y = elem.task_y;
		    task_pose.theta = elem.task_theta;
		    
		    ROS_INFO("the pose of %s is: x: %.2f, y: %.2f, theta: %.2f", task_name.c_str(), task_pose.x, task_pose.y, task_pose.theta);
		}
		else
		    publishIniStatus();
	    }   
	}
    }


};


int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "robot_node");
    
    ros::NodeHandle node;
    string name = std::string(argv[1]);
    int id = atoi(argv[2]);
    Robot robot(node, name, id);

    sleep(1); 



    ros::Rate rate(10);
    while (ros::ok()) 
    {
	while(!robot.assignment && ros::ok())
	{
	    robot.publishIniStatus();  
	    ros::spinOnce();
	    rate.sleep();
	}
	while(robot.assignment && ros::ok())
	{  
	    // il robot si muove verso il task
	    ROS_INFO_STREAM("ROBOT "<< robot.robot_name <<" IS MOVING TO " << robot.task_name);
	    robot.moveGoal(robot.task_pose, DISTANCE_TOLERANCE);
// 	    // il robot si muove verso l'uscita
// 	    ROS_INFO_STREAM("ROBOT "<< robot_name <<" IS MOVING TO THE EXIT");
// 	    moveGoal(uscita,distance_tolerance);
	    
	    robot.deleteMarker(robot.task_pose, robot.task_id_marker);
	    
	    robot.assignment=false;
	}
	
// 	robot.publishMarker();
	
	robot.publishFreeStatus();
	ros::spinOnce(); 
	rate.sleep();
    }
 
    return 0;
}
