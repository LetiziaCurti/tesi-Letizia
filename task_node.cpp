// nodo che modella l'agente task
// il task comunica il suo stato (nome, status=false(not busy), posizione) al central node 
// (che lo impila nella coda in base al suo tempo di arrivo) scrivendo su "task_arrival_topic",
// finché non riceve un assignment dal central node (legge "assignment_topic")
// Mentre è impegnato rimane in ascolto del suo stato leggendo da "assignment_topic", quando riceve dal 
// master di essere stato liberato ricomincia a pubblicare il suo stato fino ad un nuovo assignment



#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "poste_pkg/AgentStatus.h"  

using namespace std;

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}


// ros::Subscriber status_sub;
ros::Publisher status_pub;
// ros::Publisher pub;
ros::Subscriber sub;
ros::Subscriber assignment_sub;

std::string task_name;
turtlesim::Pose turtlesim_pose;

bool assignment = false;





// Callback con cui il task legge la sua posizione dal /sim node
void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    //ROS_INFO("x: %.2f, y: %.2f, theta: %.2f", msg->x, msg->y, msg->theta);
    turtlesim_pose.x = msg -> x;
    turtlesim_pose.y = msg -> y;
    turtlesim_pose.theta = msg -> theta;
}



// Il task pubblica il suo stato su "task_arrival_topic"
void publishIniStatus() 
{
  
    poste_pkg::AgentStatus status_msg;

    status_msg.header.stamp = ros::Time::now();
    status_msg.t = ros::Time::now();
    status_msg.robot_id = task_name;
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
void AssignCallback(const poste_pkg::AgentStatus::ConstPtr& status_msg)
{
    //check: deve essere arrivato qualcosa
    if(status_msg->is_ready)
    {
      	ROS_INFO_STREAM(task_name << " is listening " << status_msg->robot_id << " with robot assigned " << status_msg->robot_assign.id);
    
	// se il task che è arrivato ha come robot assegnato me, metto assignment a true così
	// smetto di pubblicare il mio stato
	if(status_msg->robot_id==task_name && status_msg->status==true)
	    assignment = true;
	else
	    assignment = false;
    }

}





int main(int argc, char **argv)
{
  
    task_name = std::string(argv[1]);
    // Initialize the node
    ros::init(argc, argv, "task_node");
    ros::NodeHandle node;
    
    turtlesim_pose.x=-1;
    turtlesim_pose.y=-1;
    turtlesim_pose.theta=200;
    
    
    // A listener for pose
    sub = node.subscribe(task_name + "/pose", 10, &poseCallback);

    // Publish and subscribe to team status messages
    status_pub = node.advertise<poste_pkg::AgentStatus>("task_arrival_topic", 10);
    
    assignment_sub = node.subscribe("assignment_topic", 20, &AssignCallback);
    sleep(1);
    
    
    publishIniStatus();
    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	while(!assignment && ros::ok())
	{
	    publishIniStatus();  
	    ros::spinOnce();
	    rate.sleep();
	}
	while(assignment && ros::ok())
	{  
	    ros::spinOnce();
	    rate.sleep();
	}
	
	publishIniStatus();
	ros::spinOnce(); 
	rate.sleep();
    }



    return 0;
}