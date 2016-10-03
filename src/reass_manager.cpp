// Nodo che gestisce il reassignment


#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>            
#include <stdlib.h>          
#include <yaml-cpp/yaml.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"
#include "task_assign/vect_robot.h"
#include "task_assign/rt_vect.h"
#include "task_assign/vect_info.h"
#include "task_assign/glpk_in.h"
#include "task_assign/glpk_sol.h"
#include "task_assign/assignment.h"




inline const char * const BoolToString(bool b)
{
    return b ? "true" : "false";
}


int Min(int a, int b)
{
    if(a<b)
	return a;
    else
	return b;
}



using namespace std;


ros::Subscriber rob_ass_sub;
ros::Subscriber rob_rech_sub;
ros::Subscriber rob_info_sub;
ros::Subscriber rech_info_sub;
ros::Subscriber task_ass_sub;
ros::Subscriber reass_sub;

ros::Publisher rt_pub;
ros::Publisher rech_pub;
ros::Publisher reass_pub;

ros::Publisher assignment_pub;
ros::Publisher recharge_pub;


vector<task_assign::task> task_to_assign;    	//T: vettore dei task da assegnare che cambia nel tempo (di dim m(k))
vector<task_assign::robot> robot_to_assign;    	//R: vettore dei robot per l'assegnazione che cambia nel tempo (di dim n(k))
vector<task_assign::info> tex_info_vect;	//vettore dei tempi di esecuzione di ciascun robot rispetto a tutti i task
vector<task_assign::info> tex0_info_vect;	//vettore dei tempi di esecuzione di ciascun robot rispetto a tutti i task
vector<task_assign::robot> delete_rob;		//robot esclusi dall'assignment perché superano le soglie
vector<task_assign::robot> robots_in_recharge;  //R: vettore dei robot che devono essere caricati
vector<task_assign::info> rech_info_vect;	

vector<task_assign::task> recharge_points;   		// è la lista di tutti i punti di ricarica presenti nello scenario, va passata dall'esterno




// Legge su "task_assign_topic" il vettore dei task da eseguire m(k)
void TaskToAssCallback(const task_assign::vect_task::ConstPtr& msg)
{
    bool add_task(true);
    
    for(auto elem : msg->task_vect)
    {
	// vedo se elem sta già in task_to_assign
	for(auto newel : task_to_assign)
	{
	    if(newel.id1 == elem.id1 && newel.id2 == elem.id2)
		add_task = false;
	}
	
	if(add_task)
	    task_to_assign.push_back(elem);
    }
}



// Legge su "rob_assign_topic" il vettore dei robot da assegnare n(k)
void RobToAssCallback(const task_assign::vect_robot::ConstPtr& msg)
{
    bool add_rob(true);
    
    for(auto elem : msg->robot_vect)
    {
	// vedo se elem sta già in robot_to_assign
	for(auto newel : robot_to_assign)
	{
	    if(newel.name == elem.name)
		add_rob = false;
	}
	
	if(add_rob)
	    robot_to_assign.push_back(elem);
    }
}



// Legge su "rob_assign_topic" il vettore dei robot da assegnare n(k)
void RobInRechCallback(const task_assign::vect_robot::ConstPtr& msg)
{
    bool add_rob(true);
    
    for(auto elem : msg->robot_vect)
    {
	// vedo se elem sta già in robot_to_assign
	for(auto newel : robots_in_recharge)
	{
	    if(newel.name == elem.name)
		add_rob = false;
	}
	
	if(add_rob)
	    robots_in_recharge.push_back(elem);
    }
}



// Legge su "rob_info_topic" le info relative ai robot (tutti: già assegnati e da assegnare)
void RobInfoCallback(const task_assign::vect_info::ConstPtr& msg)
{
    // non è importante l'ordine
    for(auto elem : msg->info_vect)
    {
	tex_info_vect.push_back(elem);
    }
}



// Legge su "rob_ini_topic" le info relative ai robot al tempo 0 (tutti: già assegnati e da assegnare)
void RechInfoCallback(const task_assign::vect_info::ConstPtr& msg)
{
    // non è importante l'ordine
    for(auto elem : msg->info_vect)
    {
	rech_info_vect.push_back(elem);
    }
}


void AssCallback(const task_assign::glpk_sol::ConstPtr& msg)
{
  
}
// // Legge "assignment_topic" 
// void RTCallback(const task_assign::rt_vect::ConstPtr& msg)
// {
//     bool add(true);
//     new_assign = false;
//     struct Assign ass;
//     
//     if(msg->rt_vect.size() > 0)
//     {	
// 	//metto le nuove coppie r-t nel catalogo, gli associo il percorso selezionandolo dalla mappa globale, metto
// 	// il robot in robots_in_execution e il task in tasks_in_execution
// 	for(auto rt : msg->rt_vect)
// 	{
// 	    //vedo se è già nel catalogo
// 	    for(auto elem : Catalogo_Ass)
// 	    {
// 		if(elem.rob.name==rt.robot.name && elem.task.name==rt.task.name)
// 		{
// 		    add = false;
// 		    break;
// 		}
// 	    }
// 	    
// 	    if(add)
// 	    {
// 		new_assign = true;
// 		
// 		robots_in_execution.push_back(rt.robot);
// 		available_robots = deleteRob(rt.robot.name, available_robots);
// 		
// 		tasks_in_execution.push_back(rt.task);
// 		tasks_to_assign = deleteTask(rt.task.name, tasks_to_assign);
// 		
// 		
// // 		ass = MapToCatal(GlobMap, rt, 1);
// 		ass = MinPath(Mappa, rt);
// 		assignments_vect.push_back(ass.path_tot);
// 		
// 		// Crea il Catalogo_Ass
// 		Catalogo_Ass.push_back(ass);
// 	    }
// 	    
// 	    add = true;
// 	}
//     }
// }
// 
// 
// 
// // Legge "recharge_topic" 
// void RechCallback(const task_assign::rt_vect::ConstPtr& msg)
// {
//     bool add(true);
//     new_in_rech = false;
//     struct Assign ass;
//     
//     if(msg->rt_vect.size() > 0)
//     {	
// 	//metto le nuove coppie r-t nel catalogo, gli associo il percorso selezionandolo dalla mappa globale, metto
// 	// il robot in robots_in_execution e il task in tasks_in_execution
// 	for(auto rt : msg->rt_vect)
// 	{
// 	    //vedo se è già nel catalogo
// 	    for(auto elem : Catalogo_Rech)
// 	    {
// 		if(elem.rob.name==rt.robot.name && elem.task.name==rt.task.name)
// 		{
// 		    add = false;
// 		    break;
// 		}
// 	    }
// 	    
// 	    if(add)
// 	    {
// 		new_in_rech = true;
// 		
// // 		ass = MapToCatal(GlobMap, rt, 0);
// 		ass = MinPath(Mappa, rt);
// 		robRech_vect.push_back(ass.path_tot);
// 		
// 		// Crea il Catalogo_Ass
// 		Catalogo_Rech.push_back(ass);
// 	    }
// 	    
// 	    add = true;
// 	}
//     }
// }



// Pubblica al master i vettori che servono al solver glpk per fare reassignment
void publishReass()
{
    task_assign::glpk_in vect_msg;

//     sleep(1);
    reass_pub.publish(vect_msg);
}



// Pubblica ai robot gli assignments task-robot
void publishRT()
{
    task_assign::rt_vect vect_msg;
    task_assign::rt msg;
    
//     sleep(1);
    rt_pub.publish(vect_msg);
}



// Pubblica ai robot gli assignments robot-rech. point
void publishRech()
{
    task_assign::rt_vect vect_msg;
    task_assign::rt msg;

    
//     sleep(1);
    rech_pub.publish(vect_msg);
}






int main(int argc, char **argv)
{
  
    // Initialize the node
    ros::init(argc, argv, "reass_manager");
    ros::NodeHandle node;
    
    rob_ass_sub = node.subscribe("rob_assign_topic", 20, &RobToAssCallback);
    rob_rech_sub = node.subscribe("rob_recharge_topic", 20, &RobInRechCallback);
    rob_info_sub = node.subscribe("rob_info_topic", 20, &RobInfoCallback);
    rech_info_sub = node.subscribe("rech_info_topic", 20, &RechInfoCallback);
    task_ass_sub = node.subscribe("task_assign_topic", 20, &TaskToAssCallback);
    
    reass_pub = node.advertise<task_assign::glpk_in>("glpk_in_topic", 10);
    reass_sub = node.subscribe("glpk_sol_topic", 20, &AssCallback);
    
    rt_pub = node.advertise<task_assign::rt_vect>("rt_topic", 10);
    rech_pub = node.advertise<task_assign::rt_vect>("rech_topic", 10);

    
    sleep(1);


    ros::Rate rate(10);
    while (ros::ok()) 
    {
	
	
	
	ros::spinOnce();
	rate.sleep();
    }
    


    return 0;
}
