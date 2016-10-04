// Nodo che modella le richieste degli utenti


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"
#include "yaml-cpp/yaml.h"


using namespace std;
ros::Publisher new_task_pub;
vector<task_assign::task> new_task_vect;
map<double, task_assign::task> map_task;



// Function con cui caricare "manualmente" i nuovi tasks, li metto in new_task_vect
void CreateNewTask()
{
    task_assign::task newTask;
    
    // Leggi tutte le info da un file yaml e mettile al posto di new_task_vect
    YAML::Node node_conf = YAML::LoadFile("/home/letizia/catkin_ws/src/task_assign/config/tasks_config.yaml");
    const YAML::Node& node_test1 = node_conf["TASK"];

    for (std::size_t i = 0; i < node_test1.size(); i++) 
    {
	const YAML::Node& node_test2 = node_test1[i];
	newTask.name = node_test2["name"].as<std::string>();
	std::cout << "Name: " << node_test2["name"].as<std::string>() << std::endl;
	newTask.ar_time = node_test2["arrt"].as<double>();
	std::cout << "Arrival time: " << node_test2["arrt"].as<double>() << std::endl;
	
	const YAML::Node& node_test3 = node_test2["tasks"];
	for (std::size_t i = 0; i < node_test3.size(); i++) 
	{
	    const YAML::Node& node_test4 = node_test3[i];
	    if(i==0)
	    {
		newTask.id1 = node_test4["id"].as<double>();
		newTask.wait1 = node_test4["wait"].as<double>();
	    }
	    else if(i==1)
	    {
		newTask.id2 = node_test4["id"].as<double>();
		newTask.wait2 = node_test4["wait"].as<double>();
	    }
	    
	    std::cout << "Id"<< i+1 << ": " << node_test4["id"].as<double>() << std::endl;
	    std::cout << "Wait: " << node_test4["wait"].as<double>() << std::endl;
	    
	    const YAML::Node& node_pos = node_test4["position"];
	    for (std::size_t j = 0; j < node_pos.size(); j++) 
	    {
		if(i==0)
		{
		    if(j==0)
		    {
			newTask.x1 = node_pos[j].as<double>();
			std::cout << "x: " << node_pos[j].as<double>() << std::endl;
		    }
		    else  if(j==1)
		    {
			newTask.y1 = node_pos[j].as<double>();
			std::cout << "y: " << node_pos[j].as<double>() << std::endl;
		    }
		    else  if(j==2)
		    {
			newTask.theta1 = node_pos[j].as<double>();
			std::cout << "theta: " << node_pos[j].as<double>() << std::endl;
		    }
		}
		else if(i==1)
		{
		    if(j==0)
		    {
			newTask.x2 = node_pos[j].as<double>();
			std::cout << "x: " << node_pos[j].as<double>() << std::endl;
		    }
		    else  if(j==1)
		    {
			newTask.y2 = node_pos[j].as<double>();
			std::cout << "y: " << node_pos[j].as<double>() << std::endl;
		    }
		    else  if(j==2)
		    {
			newTask.theta2 = node_pos[j].as<double>();
			std::cout << "theta: " << node_pos[j].as<double>() << std::endl;
		    }
		}
	    }
	}
	
	map_task[newTask.ar_time] = newTask;
	
// 	new_task_vect.push_back(newTask);
    }
}




void publishVectTask()
{
    task_assign::vect_task vect_msg;
    vect_msg.task_vect = new_task_vect;
    
    // Wait for the publisher to connect to subscribers
//     sleep(1.0);
    new_task_pub.publish(vect_msg);
    
    for(auto elem : vect_msg.task_vect)
    {
	ROS_INFO_STREAM("The users node is publishing the task: "<< elem.name << " with the couple " << elem.id1 << " - " << elem.id2);
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
    
    // carico i task dal file yaml e li metto nella mappa map_task, in cui vengono ordinati dal primo che arriva all'ultimo
    CreateNewTask();
    
    ros::Rate rate(10);
    double current_at;
    current_at = map_task.begin()->first;
    
    
    while(ros::ok())
    {
	for(map<double, task_assign::task>::iterator it=map_task.begin(); it!=map_task.end(); ++it)
	{
	    if(it->first == current_at)
		new_task_vect.push_back(it->second);
	    else
	    {
		// pubblico tutti i task con lo stesso tempo di arrivo
		sleep(current_at);
		publishVectTask();
		new_task_vect.clear();	    
		
		current_at = it->first;
		new_task_vect.push_back(it->second);
	    }  
	}
	

	ros::spinOnce();
	rate.sleep();
    }


    return 0;
}


//     // aggiungo il task1
//     newTask.ar_time = 5;
//     newTask.name = "task1";
//     newTask.name1 = "taska";
//     newTask.id1 = 1;
//     newTask.status1 = true;
//     newTask.wait1 = 3;
//     newTask.x1 = 7;
//     newTask.y1 = 8.5;
//     newTask.theta1 = -1.5;
//     newTask.name2 = "taskb";
//     newTask.id2 = 3;
//     newTask.status2 = true;
//     newTask.wait2 = 5;
//     newTask.x2 = 16;
//     newTask.y2 = 15;
//     newTask.theta2 = -1.5; 
//     new_task_vect.push_back(newTask);
//     
//     // aggiungo il task2
//     newTask.ar_time = 10;
//     newTask.name = "task2";
//     newTask.name1 = "taskc";
//     newTask.id1 = 4;
//     newTask.status1 = true;
//     newTask.wait1 = 3;
//     newTask.x1 = 5.5;
//     newTask.y1 = 5.5;
//     newTask.theta1 = -1.5;
//     newTask.name2 = "taskd";
//     newTask.id2 = 2;
//     newTask.status2 = true;
//     newTask.wait2 = 10;
//     newTask.x2 = 2;
//     newTask.y2 = 19;
//     newTask.theta2 = -1.5; 
//     new_task_vect.push_back(newTask);
//     
//     // aggiungo il task3
//     newTask.ar_time = 15;
//     newTask.name = "task3";
//     newTask.name1 = "taske";
//     newTask.id1 = 5;
//     newTask.status1 = true;
//     newTask.wait1 = 3;
//     newTask.x1 = 18;
//     newTask.y1 = 10;
//     newTask.theta1 = -1.5;
//     newTask.name2 = "taskf";
//     newTask.id2 = 6;
//     newTask.status2 = true;
//     newTask.wait2 = 15;
//     newTask.x2 = 1;
//     newTask.y2 = 1;
//     newTask.theta2 = -1.5; 
//     new_task_vect.push_back(newTask);
//     
//         // aggiungo il task4
//     newTask.ar_time = 10;
//     newTask.name = "task4";
//     newTask.name1 = "taskg";
//     newTask.id1 = 7;
//     newTask.status1 = true;
//     newTask.wait1 = 20;
//     newTask.x1 = 18;
//     newTask.y1 = 3;
//     newTask.theta1 = -1.5;
//     newTask.name2 = "taskh";
//     newTask.id2 = 8;
//     newTask.status2 = true;
//     newTask.wait2 = 5;
//     newTask.x2 = 11;
//     newTask.y2 = 17;
//     newTask.theta2 = -1.5; 
//     new_task_vect.push_back(newTask);
