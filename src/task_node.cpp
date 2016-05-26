// nodo che modella l'agente task
// il task comunica il suo stato (nome, status=false(not busy), posizione) al central node 
// (che lo impila nella coda in base al suo tempo di arrivo) scrivendo su "task_arrival_topic",
// finché non riceve un assignment dal central node (legge "assignment_topic") e poi muore

// Da aggiungere:
// Mentre è impegnato rimane in ascolto del suo stato leggendo da "assignment_topic", quando riceve dal 
// master che il suo stato è tornato a false pubblica che è stato eseguito e muore



#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "task_assign/AgentStatus.h"  

using namespace std;

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}


class Task
{
public:
  
    ros::Publisher status_pub;
    ros::Subscriber sub;
    ros::Subscriber assignment_sub;

    std::string task_name;
    turtlesim::Pose turtlesim_pose;

    bool assignment = false;


    Task(ros::NodeHandle& node, string name) 
    {
	task_name = name;
    
	turtlesim_pose.x=-1;
	turtlesim_pose.y=-1;
	turtlesim_pose.theta=200;
    
	sub = node.subscribe(task_name + "/pose", 10, &Task::poseCallback,this);

	// Publish and subscribe to team status messages
	status_pub = node.advertise<task_assign::AgentStatus>("task_arrival_topic", 10);
	
	assignment_sub = node.subscribe("assignment_topic", 20, &Task::AssignCallback,this);
    }


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
      
	task_assign::AgentStatus status_msg;

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
    void AssignCallback(const task_assign::AgentStatus::ConstPtr& status_msg)
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


};




int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "task_node");
    
    ros::NodeHandle node;
    string name = std::string(argv[1]);
    Task task(node, name);

    sleep(1);
    
    
    task.publishIniStatus();
    
    ros::Rate rate(10);

    while(!task.assignment && ros::ok())
    {
	task.publishIniStatus();  
	ros::spinOnce();
	rate.sleep();
    }
    
    // il task dovrebbe ricevere dal master che il robot a lui assegnato ha finito di eseguirlo (va aggiunto questo messaggio)
    // e dopo pubblicare che è stato eseguito e morire
//     while(assignment && ros::ok())
//     {  
// 	ros::spinOnce();
// 	rate.sleep();
//     }
//     if(!assignment)     // e qui assignment dovrebbe diventare false se è vera (status_msg->robot_id==task_name && status_msg->status==false)
// 	ROS_INFO_STREAM(task_name << " è stato eseguito");


    return 0;
}
