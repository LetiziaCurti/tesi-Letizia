// Nodo che modella le richieste degli utenti


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"
// // #include "yaml-cpp/yaml.h"


using namespace std;
ros::Publisher new_task_pub;
vector<task_assign::task> new_task_vect;



// Function con cui caricare "manualmente" i nuovi tasks, li metto in new_task_vect
void CreateNewTask()
{
    task_assign::task newTask;
    
    // aggiungo il task1
    newTask.ar_time = 5;
    newTask.name = "task1";
    newTask.name1 = "taska";
    newTask.id1 = 1;
    newTask.status1 = true;
    newTask.wait1 = 3;
    newTask.x1 = 7;
    newTask.y1 = 8.5;
    newTask.theta1 = -1.5;
    newTask.name2 = "taskb";
    newTask.id2 = 3;
    newTask.status2 = true;
    newTask.wait2 = 5;
    newTask.x2 = 16;
    newTask.y2 = 15;
    newTask.theta2 = -1.5; 
    new_task_vect.push_back(newTask);
    
    // aggiungo il task2
    newTask.ar_time = 10;
    newTask.name = "task2";
    newTask.name1 = "taskc";
    newTask.id1 = 4;
    newTask.status1 = true;
    newTask.wait1 = 3;
    newTask.x1 = 5.5;
    newTask.y1 = 5.5;
    newTask.theta1 = -1.5;
    newTask.name2 = "taskd";
    newTask.id2 = 2;
    newTask.status2 = true;
    newTask.wait2 = 10;
    newTask.x2 = 2;
    newTask.y2 = 19;
    newTask.theta2 = -1.5; 
    new_task_vect.push_back(newTask);
    
    // aggiungo il task3
    newTask.ar_time = 15;
    newTask.name = "task3";
    newTask.name1 = "taske";
    newTask.id1 = 5;
    newTask.status1 = true;
    newTask.wait1 = 3;
    newTask.x1 = 18;
    newTask.y1 = 10;
    newTask.theta1 = -1.5;
    newTask.name2 = "taskf";
    newTask.id2 = 6;
    newTask.status2 = true;
    newTask.wait2 = 15;
    newTask.x2 = 1;
    newTask.y2 = 1;
    newTask.theta2 = -1.5; 
    new_task_vect.push_back(newTask);
    
        // aggiungo il task4
    newTask.ar_time = 10;
    newTask.name = "task4";
    newTask.name1 = "taskg";
    newTask.id1 = 7;
    newTask.status1 = true;
    newTask.wait1 = 20;
    newTask.x1 = 18;
    newTask.y1 = 3;
    newTask.theta1 = -1.5;
    newTask.name2 = "taskh";
    newTask.id2 = 8;
    newTask.status2 = true;
    newTask.wait2 = 5;
    newTask.x2 = 11;
    newTask.y2 = 17;
    newTask.theta2 = -1.5; 
    new_task_vect.push_back(newTask);
}

// struct couple_task
// {
//     double ar_time;
//     string name1;
//     int id1;
//     bool status1;
//     double wait1;
//     float x1;
//     float y1;
//     float theta1;
//     
//     string name2;
//     int id2;
//     bool status2;
//     double wait2;
//     float x2;
//     float y2;
//     float theta2;
// };

// namespace YAML {
// template<>
// struct convert<task_assign::task> {
//   
//     static Node encode(const task_assign::task& rhs) {
// 	Node node;
// 	node.push_back(rhs.name);
// 	node.push_back(rhs.ar_time);
// 	node.push_back(rhs.name1);
// 	node.push_back(rhs.id1);
// 	node.push_back(rhs.x1);
// 	node.push_back(rhs.y1);
// 	node.push_back(rhs.theta1);
// 	node.push_back(rhs.wait1);
// 	node.push_back(rhs.status1);
// 	node.push_back(rhs.name2);
// 	node.push_back(rhs.id2);
// 	node.push_back(rhs.x2);
// 	node.push_back(rhs.y2);
// 	node.push_back(rhs.theta2);
// 	node.push_back(rhs.wait2);
// 	node.push_back(rhs.status2);
// 	return node;
//     }
// 
//     static bool decode(const Node& node, task_assign::task& rhs) {
// //     if(!node.IsSequence() || node.size() != 13) {
// //       return false;
// //     }
// 
// 	rhs.name = node["name"].as<string>();
// 	rhs.ar_time = node["ritardo"].as<double>();
// 	rhs.name1 = node["name1"].as<string>();
// 	rhs.id1 = node["id1"].as<int>();
// 	rhs.x1 = node["position1"][0].as<double>();
// 	rhs.y1 = node["position1"][1].as<double>();
// 	rhs.theta1 = node["position1"][2].as<double>();
// 	rhs.wait1 = node["wait1"].as<double>();
// 	rhs.status1 = node["status1"].as<bool>();
// 	rhs.name2 = node["name2"].as<string>();
// 	rhs.id2 = node["id2"].as<int>();
// 	rhs.x2 = node["position2"][0].as<double>();
// 	rhs.y2 = node["position2"][1].as<double>();
// 	rhs.theta2 = node["position2"][2].as<double>();
// 	rhs.wait2 = node["wait2"].as<double>();
// 	rhs.status2 = node["status2"].as<bool>();
// 	
// 	return true;
//     }
// };
// }



void publishVectTask()
{
    task_assign::vect_task vect_msg;
    vect_msg.task_vect = new_task_vect;
    
    // Wait for the publisher to connect to subscribers
    sleep(1.0);
    new_task_pub.publish(vect_msg);
    
    for(auto elem : vect_msg.task_vect)
    {
	ROS_INFO_STREAM("The users node is publishing the task: "<< elem.name << " whit the couple " << elem.name1 << " - " << elem.name2);
    }
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
    
    CreateNewTask();
    
    // Leggi tutte le info da un file yaml e mettile al posto di new_task_vect
//     YAML::Node node_conf = YAML::LoadFile("config.yaml");
    
    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	publishVectTask();
	ros::spinOnce();
	rate.sleep();
    }



    return 0;
}
