// Nodo che modella il motion planner


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"
#include "task_assign/vect_robot.h"
#include "task_assign/rt_vect.h"
#include "task_assign/vect_info.h"
#include "task_assign/path.h"


inline const char * const BoolToString(bool b)
{
    return b ? "true" : "false";
}

double getDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

using namespace std;


ros::Subscriber obs_sub;
ros::Subscriber assignment_sub;
ros::Subscriber status_rob_sub;
ros::Subscriber arr_rob_sub;
ros::Subscriber task_ass_sub;

ros::Publisher exec_task_pub;
ros::Publisher rob_ass_pub;
ros::Publisher rob_ini_pub;
ros::Publisher rob_info_pub;
ros::Publisher rt_pub;

#define VELOCITY 10
#define BATTERY_THR 10


vector<task_assign::robot> available_robots;    	//R: vettore dei robot per l'assegnazione che cambia nel tempo (di dim n(k))
vector<task_assign::robot> robots_in_execution;    	//vettore dei robot che stanno eseguendo dei task o che stanno andando a ricaricarsi
vector<task_assign::robot> robots_in_recharge;    	//vettore dei robot che stanno eseguendo dei task o che stanno andando a ricaricarsi
vector<task_assign::task> tasks_to_assign;    		//T: vettore dei task da assegnare che cambia nel tempo di dimensione m(k)
vector<task_assign::task> tasks_in_execution;		//vettore dei task già in esecuzione
vector<task_assign::rt> rt_vector;			//vettore degli assignments robot-task
vector<task_assign::info> tex0_info_vect;	//vettore dei tempi di esecuzione di ciascun robot rispetto a tutti i task(al tempo 0)


//simulo una mappa
struct mappa
{
    string t_name;
    string r_name;
    vector<pair<double,double>> wpoints;
};

vector<mappa> maps;   // la mappa globale è un vettore di strutture mappa, ognuna rappresenta il path per andare da un robot ad un task



// Function che elimina il robot con il nome passato in argomento dal vettore di robot passato in argomento
vector<task_assign::robot> deleteElem(string name, vector<task_assign::robot> vect)
{
    int i(0);
    for(auto elem : vect)
    {
	if(elem.name == name)
	{
	    vect.erase(vect.begin()+i);
	    break;
	}
	i++;
    }
    
  
    return vect;
}



double CalcPath(vector<pair<double,double>> wpoints)
{
    double dist(0);
    
    for(int i=0; i<wpoints.size()-1; i++)
    {
	dist += getDistance(wpoints[i].first,wpoints[i].second,wpoints[i+1].first,wpoints[i+1].second);
    }
    
    return dist;
}



vector<task_assign::info> CalcTex(vector<task_assign::robot> robots, vector<task_assign::task> tasks, vector<mappa> maps)
{
    vector<task_assign::info> tex;
    task_assign::info info;
    
    // idea 1
    for(auto rob : robots)
    {
	// vedo se elem sta già in task_to_assign
	for(auto task : tasks)
	{
	      for(auto elem : maps)
	      {
		  if(rob.name == elem.r_name && task.name == elem.t_name)
		  {
		      info.r_name = rob.name;
		      info.t_name = task.name;
		      info.t_ex = 1/VELOCITY*CalcPath(elem.wpoints);
		      tex.push_back(info);
		      
		      break;
		  }
	      }
	}
    }
    
    return tex;
}



// Legge su "task_assign_topic" il vettore dei task da eseguire T
void TaskToAssCallback(const task_assign::vect_task::ConstPtr& msg)
{
    bool add_task(true);
    
    for(auto elem : msg->task_vect)
    {
	// vedo se elem sta già in task_to_assign
	for(auto newel : tasks_to_assign)
	{
	    if(newel.name1 == elem.name1 && newel.name2 == elem.name2)
		add_task = false;
	}
	
	if(add_task)
	    tasks_to_assign.push_back(elem);
    }
}



// Legge "status_rob_topic" 
void StatusCallback(const task_assign::robot::ConstPtr& msg)
{    
    bool add_rob(true);
    bool in_charge(false);
    bool in_execution(false);
    bool available(false);
    
    if(msg->status)
    {	
	for(auto elem : available_robots)
	{
	    if(elem.name == msg->name)
	    {
		available = true;
		break;
	    }
	}
	
	if(!available)
	{
	    for(auto elem : robots_in_execution)
	    {
		if(elem.name == msg->name)
		{
		    in_execution = true;
		    break;
		}
	    }
	}
	
	if(!available && !in_execution)
	{
	    for(auto elem : robots_in_recharge)
	    {
		if(elem.name == msg->name)
		{
		    in_charge = true;
		    break;
		}
	    }
	}
	
   
	if(!available && !in_execution && !in_charge && msg->b_level > BATTERY_THR)
	{
	    available_robots.push_back(*msg);
	}
	
	else if(!available && !in_execution && !in_charge && msg->b_level <= BATTERY_THR)
	{
	    robots_in_recharge.push_back(*msg);
	}
	
	else if(available)
	{
	    if(msg->b_level <= BATTERY_THR)
	    {
		    robots_in_recharge.push_back(*msg);
		    available_robots = deleteElem(msg->name,available_robots);
	    }
	}
	
	else if(in_execution)
	{
	    if(msg->b_level <= BATTERY_THR)
	    {
		    // comunicalo al master in qualche modo
	    }
	}
	
	else if(in_charge)
	{
	    if(msg->b_level == msg->b_level0)
		robots_in_recharge = deleteElem(msg->name,robots_in_recharge);
	}
	


    }
}



// // Pubblica al master su "rob_info_topic" le info relative ai robot (tutti: già assegnati e da assegnare)
// void publishRobotIni()
// {
//     task_assign::vect_info vect_msg;
// 
// }




// Pubblica al master su "rob_info_topic" le info relative ai robot (tutti: già assegnati e da assegnare), pubblica tex0_info_vect
void publishRobotInfo()
{
    task_assign::vect_info vect_msg;
    
    vect_msg.info_vect = CalcTex(available_robots, tasks_to_assign, maps);
//     CalcTex(not_available_robots, tasks_to_assign, maps);
    
    sleep(1);
    rob_info_pub.publish(vect_msg);
   
}



// // Legge la posizione e il livello di batteria dei robot da "status_rob_topic" 
// void StatusCallback(const task_assign::robot::ConstPtr& msg)
// {
// 
// }


// Pubblica al master su "rob_assign_topic" il vettore dei robot da assegnare
void publishRobotToAssign()
{
    task_assign::vect_robot vect_msg;
    
    vect_msg.robot_vect = available_robots;
    
    sleep(1);
    rob_ass_pub.publish(vect_msg);
}



// Pubblica al task manager i task completati su "task_exec_topic"
void publishExecTask()
{
    task_assign::vect_task vect_msg;
}


// Pubblica ai robot i task rispettivamente assegnati
void publishRTAssign()
{
    task_assign::vect_task vect_msg;
}



// Legge "obstacles_topic" 
void ObsCallback(const task_assign::vect_task::ConstPtr& msg)
{

}



// Legge "assignment_topic" 
void AssignCallback(const task_assign::rt_vect::ConstPtr& msg)
{

}



int main(int argc, char **argv)
{
  
    // Initialize the node
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle node;
    
    obs_sub = node.subscribe("obstacles_topic", 20, &ObsCallback);
    assignment_sub = node.subscribe("assignment_topic", 20, &AssignCallback);
//     arr_rob_sub = node.subscribe("status_rob_topic", 20, &ArrCallback);
    status_rob_sub = node.subscribe("status_rob_topic", 20, &StatusCallback);
    task_ass_sub = node.subscribe("task_assign_topic", 20, &TaskToAssCallback);
    
    exec_task_pub = node.advertise<task_assign::vect_task>("task_exec_topic", 10);   
    rob_ass_pub = node.advertise<task_assign::vect_robot>("rob_assign_topic", 10);
//     rob_ini_pub = node.advertise<task_assign::vect_info>("rob_ini_topic", 10);
    rob_info_pub = node.advertise<task_assign::vect_info>("rob_info_topic", 10);
    rt_pub = node.advertise<task_assign::vect_task>("rt_topic", 10);
    
    sleep(1);

    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	ros::spinOnce();
	rate.sleep();
    }



    return 0;
}




    // deve monitorare il b_lev, se scende al di sotto della soglia il robot deve essere tolto dal vettore
    
//     if(t_ex > t_ex_0)
// 		b_lev += (t_ex - t_ex_0)*1;
// 	    else
// 		b_lev++;
// 	    
// 	    robot.b_level = b_lev;
