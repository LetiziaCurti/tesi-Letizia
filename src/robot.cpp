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

struct pose{
    float x;
    float y;
    float theta;  
};


class Robot
{
public:
  
    std::string robot_name;
    int id_marker;
    struct pose turtlesim_pose;

    ros::Subscriber assignment_sub;
    ros::Subscriber recharge_sub;
    ros::Publisher status_pub;
    ros::Publisher marker_pub;

    ros::Time t_arrive;
    struct pose task_pose;

    bool assignment = false;
    string task_name;
    int task_id_marker;
    

    Robot(ros::NodeHandle& node, string name, int id, struct pose pos) 
    {
	robot_name = name;
	id_marker = id;
	
	turtlesim_pose.x = pos.x;
	turtlesim_pose.y = pos.y;
	turtlesim_pose.theta = pos.theta;
	

	// Publish and subscribe to team status messages
	status_pub = node.advertise<task_assign::robot>("status_rob_topic", 10);	
	
	assignment_sub = node.subscribe("assignment_topic", 20, &Robot::AssignCallback,this);
	recharge_sub = node.subscribe("recharge_topic", 20, &Robot::RechargeCallback,this);
	
	marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }

    
    void publishMarker(struct pose p)
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
 
 
 
    // Il robot pubblica il suo stato su "robot_arrival_topic"
    void publishIniStatus() 
    {
	task_assign::robot status_msg; 

	status_msg.header.stamp = ros::Time::now();
// 	status_msg.t = ros::Time::now();
	status_msg.name = robot_name;
	status_msg.status = true;
	status_msg.b_level0 = 10;
	status_msg.b_level = 10;
	
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
    
    
    void deleteMarker(struct pose task_pose, int t_id)
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
    
        
    // Function con cui viene data al robot la posizione passata in argomento, che viene poi inviata a tf
    void broadcastPose(struct pose posa, std::string name)
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


    // Function for bringing the robot in the position of the task to accomplish and then in the position of
    // the exit
    void moveGoal(struct pose goal_pose, string goal_name, double distance_tolerance)
    {
	float pos_x = 0.0;
	float pos_y = 0.0;
	float theta = 0.0;
	
	double vel_x;
	double vel_z;
	double time = 0.1;
	
	
	ros::Rate rate(10);
	do{
	      publishMarker(turtlesim_pose);
	      broadcastPose(turtlesim_pose,robot_name);
	      
	      vel_x = 0.5*getDistance(turtlesim_pose.x,turtlesim_pose.y,task_pose.x,task_pose.y);
	      vel_z = 4*sin((atan2(task_pose.y - turtlesim_pose.y, task_pose.x - turtlesim_pose.x)-turtlesim_pose.theta));
	      
	      turtlesim_pose.x = (vel_x*cos(turtlesim_pose.theta))*time + turtlesim_pose.x;
	      turtlesim_pose.y = (vel_x*sin(turtlesim_pose.theta))*time + turtlesim_pose.y;
	      turtlesim_pose.theta = sin(vel_z*time) + turtlesim_pose.theta;	

	      ros::spinOnce();
	      rate.sleep(); 
	}while(getDistance(turtlesim_pose.x,turtlesim_pose.y,task_pose.x,task_pose.y)>distance_tolerance);
	
	deleteMarker(task_pose, task_id_marker);

    }



//     // Il robot pubblica su "assignment_topic" un messaggio contenente lo stato del task che ha eseguito
//     // e il suo stato (nel campo msg.robot_assign), rimettendo a false gli status suo e del task eseguito
//     void publishFreeStatus() 
//     {
// 	task_assign::OneAssign status_msg; 
// 
// 	status_msg.task_id = task_name;
// 	status_msg.t_ready = true;
// 	status_msg.t_status = true; //tengo lo stato del task a true (anche se dovrebbe essere comunicato false al task per fargli capire che è stato terminato)
// 	status_msg.task_x = task_pose.x;
// 	status_msg.task_y = task_pose.y;
// 	status_msg.task_theta = task_pose.theta;
// 	
// 	//pubblico robot_assign
// 	status_msg.rob_id = robot_name;
// 	status_msg.r_status = false;  //rimetto il mio stato su false
// 
// 
// 	// Wait for the publisher to connect to subscribers
// // 	sleep(1.0);
// 	assignment_pub.publish(status_msg);
// 	
// 	ROS_INFO_STREAM(robot_name <<" has finished the assignment. So "<< task_name << " is executed.");
//     }




    // Il robot legge "assignment_topic" aspettando di ricevere un assignment
    // Pubblica il proprio stato di ready su "robots_arrival_topic" finché non riceve un assignment, 
    // dopodiché smette di pubblicare il suo stato e memorizza la posizione del task da raggiungere
    // in task_pose
    void AssignCallback(const task_assign::assignment::ConstPtr& status_msg)
    {
	if(assignment) return;
	
	for(auto elem : status_msg->assign_vect)
	{
// 	    //check: deve essere arrivato qualcosa
// 	    if(elem.t_ready && elem.task_x!=0 && elem.task_y!=0 && elem.task_theta!=0)
// 	    {
// // 		ROS_INFO_STREAM(robot_name << " is listening " << status_msg->robot_id << " with robot assigned " << status_msg->robot_assign.id);
// 	    
// 		// se il task che è arrivato ha come robot assegnato me, metto assignment a true così
// 		// smetto di pubblicare il mio stato
// 		if(elem.rob_id==robot_name && elem.r_status==true)
// 		{
// 		    assignment = true;
// 		    task_name = elem.task_id;
// 		    task_id_marker = elem.t_id_marker;
// 		    
// 		    task_pose.x = elem.task_x;
// 		    task_pose.y = elem.task_y;
// 		    task_pose.theta = elem.task_theta;
// 		    
// 		    ROS_INFO("the pose of %s is: x: %.2f, y: %.2f, theta: %.2f", task_name.c_str(), task_pose.x, task_pose.y, task_pose.theta);
// 		}
// 		else
// 		    publishIniStatus();
// 	    }   
	}
    }
    
    
    
    void RechargeCallback(const task_assign::vect_robot::ConstPtr& status_msg)
    {
	
    }


};


int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "robot_node");
    
    ros::NodeHandle node;
    string name = std::string(argv[1]);
    int id = atoi(argv[2]);
    
    struct pose pose;
    pose.x = atof(argv[3]);
    pose.y = atof(argv[4]);
    pose.theta = atof(argv[5]);
    Robot robot(node, name, id, pose);

    sleep(1); 



    ros::Rate rate(10);
    while (ros::ok()) 
    {
	while(!robot.assignment && ros::ok())
	{
	    robot.broadcastPose(robot.turtlesim_pose, name);
	    robot.publishIniStatus();  
	    ros::spinOnce();
	    rate.sleep();
	}
	while(robot.assignment && ros::ok())
	{  
	    // il robot si muove verso il task
	    ROS_INFO_STREAM("ROBOT "<< robot.robot_name <<" IS MOVING TO " << robot.task_name);
	    robot.moveGoal(robot.task_pose, robot.task_name, DISTANCE_TOLERANCE);
	    
	    robot.assignment=false;
	}
	
	robot.broadcastPose(robot.turtlesim_pose, name);
// 	robot.publishFreeStatus();
	ros::spinOnce(); 
	rate.sleep();
    }
 
    return 0;
}
