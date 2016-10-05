// Nodo che modella le richieste degli utenti


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"
#include "task_assign/waypoint.h"
#include "yaml-cpp/yaml.h"


using namespace std;
ros::Publisher new_task_pub;
ros::Publisher marker_pub;

vector<task_assign::task> new_task_vect;
vector<task_assign::task> markers;
map<double, task_assign::task> map_task;





// i task sono cilindri viola
void publishMarkerPair(task_assign::waypoint p1, task_assign::waypoint p2, int id_marker1, int id_marker2)
{
    visualization_msgs::MarkerArray markers_vect;
    
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "robot_node";
    marker.id = id_marker1;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = p1.x;
    marker.pose.position.y = p1.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = p1.theta;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.5f;
    marker.color.g = id_marker1*0.01f;
    marker.color.b = 0.8f;
    marker.color.a = 0.7;

    marker.lifetime = ros::Duration();
    
    markers_vect.markers.push_back(marker);

    

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "robot_node";
    marker.id = id_marker2;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = p2.x;
    marker.pose.position.y = p2.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = p2.theta;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.5f;
    marker.color.g = id_marker1*0.01f;
    marker.color.b = 0.8f;
    marker.color.a = 0.7;

    marker.lifetime = ros::Duration();
    
    markers_vect.markers.push_back(marker);
    

    // Publish the markers
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
	break;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(markers_vect);
}



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


void VectMarker(vector<task_assign::task> markers)
{
    task_assign::waypoint p1;
    task_assign::waypoint p2;
    
    for(auto elem : markers)
    {
	p1.x = elem.x1;
	p1.y = elem.y1;
	p1.theta = elem.theta1;
	p2.x = elem.x2;
	p2.y = elem.y2;
	p2.theta = elem.theta2;
	publishMarkerPair(p1, p2, elem.id1, elem.id2);
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
    marker_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
	
    sleep(1);
    
    // carico i task dal file yaml e li metto nella mappa map_task, in cui vengono ordinati dal primo che arriva all'ultimo
    CreateNewTask();
    
    markers.clear();
    vector<task_assign::task> perm;
    ros::Rate rate(10);
    double prec_at(0.0);
    double pre_prec_at(0.0);
    double current_at;
    current_at = map_task.begin()->first;
    map<double, task_assign::task>::iterator current_it;
    current_it = map_task.begin();
    
    while(ros::ok() && map_task.size()>markers.size())
    {
	VectMarker(markers);
	
	map<double, task_assign::task>::iterator it = current_it;
	while(it!=map_task.end() && ros::ok())
	{
	    VectMarker(markers);
	    
	    if(it->first == current_at)
	    {
		new_task_vect.push_back(it->second);
		perm.push_back(it->second);
	    }
	    else
	    {	     
		pre_prec_at = prec_at;
		prec_at = current_at;
		current_at = it->first;
		current_it = it;
		break;
	    }  
	    ++it;
	}
	
	
	int i = prec_at-pre_prec_at;
	while(i>0 && ros::ok())
	{
	    VectMarker(markers);
	    sleep(1);
	    i--;
	}
	
	for(auto elem : perm)
	{	
	    markers.push_back(elem);
	  
	}
	perm.clear();
	    
	VectMarker(markers);
	publishVectTask();
	new_task_vect.clear();

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
