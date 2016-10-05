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
map<double, task_assign::task> map_task;

struct MyMarker
{
    task_assign::task task;
    vector<float> rgb;
};
vector<MyMarker> markers;




vector<float> gen3rand()
{
    vector<float> rgb;
    float r,g,b;
    r = rand()%100 + 1;
    rgb.push_back(r);
    
    g = r;
    b = g;

    while(g == r)
    {
	g = rand()%100 + 1;
    }
    rgb.push_back(g);
    while(b == g)
    {
	b = rand()%100 + 1;
    }
    rgb.push_back(b);
    
    return rgb;
}


// i task sono cilindri viola
void publishMarkerPair(task_assign::waypoint p1, task_assign::waypoint p2, int id_marker1, int id_marker2, vector<float> rgb)
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
    marker.color.r = rgb[0]*0.01f;
    marker.color.g = rgb[1]*0.01f;
    marker.color.b = rgb[2]*0.01f;
    marker.color.a = 1;

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
    marker.color.r = rgb[0]*0.01f;
    marker.color.g = rgb[1]*0.01f;
    marker.color.b = rgb[2]*0.01f;
    marker.color.a = 0.2;

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
    }
}


void VectMarker(vector<MyMarker> markers)
{
    task_assign::waypoint p1;
    task_assign::waypoint p2;
    vector<float> colours;
    
    for(auto elem : markers)
    {
	p1.x = elem.task.x1;
	p1.y = elem.task.y1;
	p1.theta = elem.task.theta1;
	p2.x = elem.task.x2;
	p2.y = elem.task.y2;
	p2.theta = elem.task.theta2;
	colours = elem.rgb;
	publishMarkerPair(p1, p2, elem.task.id1, elem.task.id2, colours);
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
    
    vector<float> rgb;
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
	
	rgb = gen3rand();
	MyMarker mark;
	for(auto elem : perm)
	{
	    if(ros::ok())
	    {
		mark.task = elem;
		mark.rgb = rgb;
		markers.push_back(mark);
	    }	  
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
