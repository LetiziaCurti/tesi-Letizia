// Nodo che modella gli ostacoli e gli imprevisti

// inoltre il nodo simula la rottura dei robot, inviando ai robot interessati il messaggio di rottura



#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "task_assign/vect_task.h"
#include "task_assign/waypoint.h"
#include <visualization_msgs/MarkerArray.h>
#include "yaml-cpp/yaml.h"


using namespace std;
ros::Publisher marker_pub;
ros::Publisher obs_pub;

vector<task_assign::task> obs_vect;
map<double, vector<task_assign::task>> map_task;


vector<task_assign::task> markers;


// i punti di ricarica sono cubi verdi
void publishMarker(task_assign::waypoint p, int id_marker, double duration)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "obstacle_node";
    marker.id = id_marker;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;
//     marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//     marker.mesh_resource = "package://task_assign/config/wall.stl";

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
//     marker.color.r = 1.0f;
//     marker.color.g = 1.0f;
//     marker.color.b = 1.0f;
    marker.color.a = 0.7;

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



// Function con cui caricare "manualmente" i nuovi tasks, li metto in new_task_vect
void CreateNewObs(string file)
{
    task_assign::task newTask;
    bool inmap(false);
    
    // Leggi tutte le info da un file yaml e mettile al posto di new_task_vect
    YAML::Node node_conf = YAML::LoadFile(file);
    const YAML::Node& node_test1 = node_conf["OBSTACLES"];

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

	map_task[newTask.ar_time].push_back(newTask);
    }
    
//     for(auto el : map_task)
//     {
//       std::cout << "task con a_time " << el.first << " :" << std::endl;
// 	for(auto elem : el.second)
// 	{
// 	    std::cout << elem.name << std::endl;
// 	}
//     }
}


void VectMarker(vector<task_assign::task> markers)
{
    if(markers.size()>0)
    {
	task_assign::waypoint p1;
	
	for(auto elem : markers)
	{
	    p1.x = elem.x1;
	    p1.y = elem.y1;
	    p1.theta = elem.theta1;
	    publishMarker(p1, elem.id1, elem.wait1);
	}
    }
}


void publishObstacles()
{
    task_assign::vect_task vect_msg;
    
    vect_msg.task_vect = obs_vect;
    
    
    // Wait for the publisher to connect to subscribers
    sleep(1.0);
    obs_pub.publish(vect_msg);
    
    for(auto el : obs_vect)
    {
	ROS_INFO_STREAM("OBS NODE sta pubblicando "<< el.name);
    }
}






int main(int argc, char **argv)
{
  
    // Initialize the node
    ros::init(argc, argv, "obstacles");
    ros::NodeHandle node;
    
    obs_pub = node.advertise<task_assign::vect_task>("obstacles_topic", 10);
    string file = std::string(argv[1]);
	
    
    marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	
    sleep(1);
    
    // carico i task dal file yaml e li metto nella mappa map_task, in cui vengono ordinati dal primo che arriva all'ultimo
    CreateNewObs(file);
    
    
    vector<task_assign::task> perm;
    ros::Rate rate(10);
    double prec_at(0.0);
    double pre_prec_at(0.0);
    double current_at = map_task.begin()->first;

    map<double, vector<task_assign::task>>::iterator it = map_task.begin();  
    while(it!=map_task.end() && ros::ok())
    {

// 	VectMarker(markers);

	for(auto elem : it->second)
	{
	    if(ros::ok())
		obs_vect.push_back(elem);	  
	}

	pre_prec_at = prec_at;
	prec_at = current_at;
	ros::spinOnce();	
	
	int i = prec_at-pre_prec_at;
	while(i>0 && ros::ok())
	{
// 	    VectMarker(markers);
	    sleep(1);
	    i--;
	    ros::spinOnce();
	}
	
	for(auto elem : it->second)
	{
	    if(ros::ok())
	    {
		markers.push_back(elem);
	    }	  
	}
	    

// 	VectMarker(markers);
	publishObstacles();
	
	++it;
	current_at = it->first;

	ros::spinOnce();
	rate.sleep();
    }
    
    while(ros::ok())
    {
//       	VectMarker(markers);
	publishObstacles();
	ros::spinOnce();
	rate.sleep();
    }


    return 0;
}