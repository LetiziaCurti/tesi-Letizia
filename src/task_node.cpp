// nodo che modella l'agente task
// il task comunica il suo stato (nome, status=false(not busy), posizione) al central node 
// (che lo impila nella coda in base al suo tempo di arrivo) scrivendo su "task_arrival_topic",
// finché non riceve un assignment dal central node (legge "assignment_topic") e poi muore



#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"  
#include "task_assign/IniStatus.h"
#include "task_assign/OneAssign.h"
#include "task_assign/AssignMsg.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

using namespace std;

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}

struct pose{
    float x;
    float y;
    float theta;  
};

class Task
{
public:
  
    ros::Publisher status_pub;
    ros::Subscriber sub;
    ros::Subscriber assignment_sub;
    ros::Publisher marker_pub;

    std::string task_name;
    int id_marker;
    turtlesim::Pose turtlesim_pose;

    bool assignment = false;


    Task(ros::NodeHandle& node, string name, int id, struct pose pos) 
    {
	task_name = name;
	id_marker = id;
    
	turtlesim_pose.x = pos.x;
	turtlesim_pose.y = pos.y;
	turtlesim_pose.theta = pos.theta;
    
// 	sub = node.subscribe(task_name + "/pose", 10, &Task::poseCallback,this);

	// Publish and subscribe to team status messages
	status_pub = node.advertise<task_assign::IniStatus>("task_arrival_topic", 10);
	
	assignment_sub = node.subscribe("assignment_topic", 20, &Task::AssignCallback,this);
	marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }

    
    // Function con cui viene data al robot la posizione passata in argomento, che viene poi inviata a tf
    void broadcastPose(struct pose p)
    {
	//ROS_INFO("x: %.2f, y: %.2f, theta: %.2f", msg->x, msg->y, msg->theta);
	
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(p.x, p.y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, p.theta);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", task_name));
	
	publishMarker();
    }
    
    
    void publishMarker()
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
	marker.type = visualization_msgs::Marker::CUBE;

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


    // Il task pubblica il suo stato su "task_arrival_topic"
    void publishIniStatus() 
    {
      
	task_assign::IniStatus status_msg;

	status_msg.header.stamp = ros::Time::now();
	status_msg.t = ros::Time::now();
	status_msg.robot_id = task_name;
	status_msg.id = id_marker;
	status_msg.is_ready = true;
	status_msg.status = false; //I am available (not busy)
	status_msg.type = "task";
	status_msg.x = turtlesim_pose.x;
	status_msg.y = turtlesim_pose.y;
	status_msg.theta = turtlesim_pose.theta;

	// Wait for the publisher to connect to subscribers
	sleep(1);
	status_pub.publish(status_msg);

	ROS_INFO_STREAM("Task "<< task_name << " is publishing its initial status " << BoolToString(status_msg.status));
    }



    // Il task legge "assignment_topic" aspettando di ricevere un assignment
    // Pubblica il proprio stato di ready su "task_arrival_topic" finché non riceve un assignment, 
    // dopodiché smette di pubblicare il suo stato mettendo assignment a true
    // Quando riceve un messaggio dal master in cui il suo stato e tornato a false, vuol dire che il robot
    // a cui era stato assegnato ha finito di eseguirlo, assignment torna false e (nel main) il task 
    // ricomincia a pubblicare il suo stato
    void AssignCallback(const task_assign::AssignMsg::ConstPtr& status_msg)
    {
	for(auto elem : status_msg->assign_vect)
	{
	    //check: deve essere arrivato qualcosa
	    if(elem.t_ready)
	    {
// 		ROS_INFO_STREAM(task_name << " is listening " << elem.task_id << " with robot assigned " << elem.rob_id);
	    
		// se il task che è arrivato ha come robot assegnato me, metto assignment a true così
		// smetto di pubblicare il mio stato
		if(elem.task_id==task_name && elem.t_status==true)
		    assignment = true;
		else
		    assignment = false;
	    }
	}

    }


};




int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "task_node");
    
    ros::NodeHandle node;
    string name = std::string(argv[1]);
    int id = atoi(argv[2]);
    int ritardo = atoi(argv[3]);
    sleep(ritardo);
    struct pose pose;
    pose.x = atof(argv[4]);
    pose.y = atof(argv[5]);
    pose.theta = atof(argv[6]);

        
    Task task(node, name, id, pose);

    sleep(1);
    
    
    task.publishIniStatus();
    
    ros::Rate rate(10);

    while(!task.assignment && ros::ok())
    {
	task.broadcastPose(pose);
	task.publishIniStatus();  
	ros::spinOnce();
	rate.sleep();
    }


    return 0;
}
