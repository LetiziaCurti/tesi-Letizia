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
#include "task_assign/task_path.h"
#include "task_assign/assignment.h"


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
ros::Subscriber rt_sub;
ros::Subscriber status_rob_sub;
ros::Subscriber arr_rob_sub;
ros::Subscriber task_ass_sub;

ros::Publisher exec_task_pub;
ros::Publisher rob_ass_pub;
ros::Publisher rob_ini_pub;
ros::Publisher rob_info_pub;
ros::Publisher assignment_pub;

#define VELOCITY 10
#define BATTERY_THR 10
bool new_assign(false);
bool add(false);


vector<task_assign::robot> available_robots;    	//R: vettore dei robot per l'assegnazione che cambia nel tempo (di dim n(k))
vector<task_assign::robot> robots_in_execution;    	//vettore dei robot che stanno eseguendo dei task o che stanno andando a ricaricarsi
vector<task_assign::robot> robots_in_recharge;    	//vettore dei robot che stanno eseguendo dei task o che stanno andando a ricaricarsi
vector<task_assign::task> tasks_to_assign;    		//T: vettore dei task da assegnare che cambia nel tempo di dimensione m(k)
vector<task_assign::task> tasks_in_execution;		//vettore dei task già in esecuzione
vector<task_assign::rt> rt_vector;			//vettore degli assignments robot-task
vector<task_assign::info> tex0_info_vect;		//vettore dei tempi di esecuzione di ciascun robot rispetto a tutti i task(al tempo 0)
vector<task_assign::task> completed_tasks;		//vettore dei task completati che viene inviato al task_manager
vector<task_assign::task_path> assignments_vect;			//vettore delle info da inviare ai robots (nome del task e percorso per raggiungerlo)



//simulo una mappa
struct mappa
{
    string t_name;
    string r_name;
    vector<pair<double,double>> wpoints;
};

vector<mappa> GlobMap;   // la mappa globale è un vettore di strutture mappa, ognuna rappresenta il path per andare da un robot ad un task



struct Assign
{
    task_assign::robot rob;
    task_assign::task task;
    task_assign::task_path path_tot;
};

vector<Assign> Catalogo_Ass;



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
    
    vect_msg.info_vect = CalcTex(available_robots, tasks_to_assign, GlobMap);
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



// Legge "assignment_topic" 
void RTCallback(const task_assign::rt_vect::ConstPtr& msg)
{
    new_assign = false;
    add = true;
    struct Assign ass;
    
    if(msg->rt_vect.size() > 0)
    {
// 	rt_vector = msg->rt_vect;
	new_assign = true;
	
	//metto le nuove coppie r-t nel catalogo, gli associo il percorso selezionandolo dalla mappa globale, metto
	// il robot in robots_in_execution e il task in tasks_in_execution
	for(auto rt : msg->rt_vect)
	{
	    //vedo se è già nel catalogo
	    for(auto elem : Catalogo_Ass)
	    {
		if(elem.rob.name==rt.robot.name && elem.task.name==rt.task.name)
		{
		    add = false;
		    break;
		}
	    }
	    
	    if(add)
	    {
		robots_in_execution.push_back(rt.robot);
		tasks_in_execution.push_back(rt.task);
		
		ass.rob = rt.robot;
		ass.task = rt.task;
		
		Catalogo_Ass.push_back(ass);
	    }
	}
    }
//     bool add(true);
//     
//     for(auto rt : msg->rt_vect)
//     {
// 	for(auto rob : robots_in_execution)
// 	{
// 	    if(rob.name == rt.r_name)
// 	    {
// 		add = false;
// 		break;
// 	    }
// 	}
// 	if(add)
// 	{}
//     }
}


// if(new_assign) ....pubblica
// Pubblica ai robot i task rispettivamente assegnati
void publishAssign()
{
    task_assign::assignment msg;
    
    msg.assign_vect = assignments_vect;
    
    sleep(1);
    assignment_pub.publish(msg);
}



// Pubblica al task manager i task completati su "task_exec_topic"
void publishExecTask()
{
    task_assign::vect_task msg;
    
    msg.task_vect = completed_tasks;
    
    sleep(1);
    exec_task_pub.publish(msg);
}






// Legge "obstacles_topic" 
void ObsCallback(const task_assign::vect_task::ConstPtr& msg)
{

}



void MapToCatal(vector<mappa> Map, Assign ass, task_assign::rt r_t)
{}






int main(int argc, char **argv)
{
  
    // Initialize the node
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle node;
    
    obs_sub = node.subscribe("obstacles_topic", 20, &ObsCallback);
    rt_sub = node.subscribe("rt_topic", 20, &RTCallback);
//     arr_rob_sub = node.subscribe("status_rob_topic", 20, &ArrCallback);
    status_rob_sub = node.subscribe("status_rob_topic", 20, &StatusCallback);
    task_ass_sub = node.subscribe("task_assign_topic", 20, &TaskToAssCallback);
    
    exec_task_pub = node.advertise<task_assign::vect_task>("task_exec_topic", 10);   
    rob_ass_pub = node.advertise<task_assign::vect_robot>("rob_assign_topic", 10);
//     rob_ini_pub = node.advertise<task_assign::vect_info>("rob_ini_topic", 10);
    rob_info_pub = node.advertise<task_assign::vect_info>("rob_info_topic", 10);
    assignment_pub = node.advertise<task_assign::assignment>("assignment_topic", 10);
    
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
