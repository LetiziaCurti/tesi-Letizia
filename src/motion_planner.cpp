// Nodo che modella il motion planner


#include <iostream>
#include <vector>
#include <string>
#include <map>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"
#include "task_assign/vect_robot.h"
#include "task_assign/rt_vect.h"
#include "task_assign/vect_info.h"
#include "task_assign/task_path.h"
#include "task_assign/assignment.h"
#include "task_assign/rech_vect.h"


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
ros::Publisher recharge_pub;

#define VELOCITY 10
#define BATTERY_THR 10
#define SEC_DIST 1

bool new_assign(false);


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
    pair<double,double> start;
    pair<double,double> end;
    vector<pair<double,double>> wpoints;
};

vector<mappa> GlobMap;   					// la mappa globale è un vettore di strutture mappa, ognuna rappresenta il path per andare da un robot ad un task
vector<task_assign::recharge> Recharge;   			// è la lista di tutti i punti di ricarica presenti nello scenario
map<task_assign::robot, task_assign::recharge> Min_Recharge;	// è la mappa che associa i robot ai punti di ricarica a distanza minima	



struct Assign
{
    task_assign::robot rob;
    task_assign::task task;
    task_assign::task_path path_tot;
};

vector<Assign> Catalogo_Ass;



// Function che elimina il robot con il nome passato in argomento dal vettore di robot passato in argomento
vector<task_assign::robot> deleteRob(string name, vector<task_assign::robot> vect)
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



// Function che elimina il robot con il nome passato in argomento dal vettore di robot passato in argomento
vector<task_assign::task> deleteTask(string name, vector<task_assign::task> vect)
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
    double time_a(0);
    double time_b(0);
    
    // idea 1
    for(auto rob : robots)
    {
	// vedo se elem sta già in task_to_assign
	for(auto task : tasks)
	{
	    info.r_name = rob.name;
	    info.t_name = task.name;
	    
	    for(auto elem : maps)
	    {
		if(rob.x == elem.start.first && rob.y == elem.start.second && task.x1 == elem.end.first && task.y1 == elem.end.second)
		{
		    time_a = 1/VELOCITY*CalcPath(elem.wpoints);
		    break;
		}
	    }
	    for(auto elem : maps)
	    {
		if(task.x1 == elem.start.first && task.y1 == elem.start.second && task.x2 == elem.end.first && task.y2 == elem.end.second)
		{
		    time_b = 1/VELOCITY*CalcPath(elem.wpoints);
		    break;
		}
	    }
	    
	    info.t_ex = time_a + task.wait1 + time_b + task.wait1;
	    tex.push_back(info);
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
    bool completed(false);
    
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
		    available_robots = deleteRob(msg->name, available_robots);
	    }
	}
	
	else if(in_execution)
	{
	    // cerco il task corrispondente al robot
	    for(auto ass : Catalogo_Ass)
	    {
		if(msg->name == ass.rob.name)
		{
		    // vedo se il robot è arrivato al task
		    if(msg->x == ass.task.x2 && msg->y == ass.task.y2)
		    {
			completed_tasks.push_back(ass.task);
			tasks_in_execution = deleteTask(ass.task.name, tasks_in_execution);
			robots_in_execution = deleteRob(msg->name, robots_in_execution);
			
			if(msg->b_level > BATTERY_THR)
			    available_robots.push_back(*msg);
			else
			    robots_in_recharge.push_back(*msg);
			
			completed = true;
			break;
		    }

		
		    // il robot sta eseguendo il task e non ha ancora finito
		    if(!completed)
		    {
			if(msg->b_level <= BATTERY_THR)
			{
			    // se manca "poco" al task (distanza inferiore ad una certa soglia), ce lo faccio arrivare e poi lo manderò in carica
			    // altrimenti mando subito il robot in carica e rimetto il task tra i task da assegnare  
			    if(getDistance(msg->x, msg->y, ass.task.x2, ass.task.y2) > SEC_DIST)
			      {
				  robots_in_recharge.push_back(*msg);
				  tasks_in_execution = deleteTask(ass.task.name, tasks_in_execution);
				  tasks_to_assign.push_back(ass.task);
			      }    
			}
		    }
		}
		break;
	    }
	}
	
	else if(in_charge)
	{
	    if(msg->b_level == msg->b_level0)
		robots_in_recharge = deleteRob(msg->name, robots_in_recharge);
	}
	


    }
}



// Function che copia una lista di w.p. dalla mappa globale in una lista di oggetti task_assign/waypoint
vector<task_assign::waypoint> CopiaPath(vector<pair<double,double>> wpoints)
{
    task_assign::waypoint wp;
    vector<task_assign::waypoint> path;
    
    for(auto pair : wpoints)
    {
	wp.x = pair.first;
	wp.y = pair.second;
	
	path.push_back(wp);
    }
    
    
    return path;
}



// Function che mette in un oggetto di tipo Assign (che verrà messo poi nel Catalogo_Ass) il robot e il task di un assignment 
// e il percorso che deve fare il robot per raggiungere il task (prima fino task_a e poi da task_a a task_b)
Assign MapToCatal(vector<mappa> Map, task_assign::rt r_t)
{
    vector<task_assign::waypoint> path;
    task_assign::waypoint wp;
    Assign ass;
  
  
    ass.rob = r_t.robot;
    ass.task = r_t.task;
    
    ass.path_tot.r_name = r_t.robot.name;
    ass.path_tot.t_name = r_t.task.name;
    
    //ora scrivo path_a prendendo dalla mappa la lista di waypoint che vanno dalla posizione del robot alla posizione di task_a di r_t
    for(auto elem : Map)
    {
	if(r_t.robot.x == elem.start.first && r_t.robot.y == elem.start.second && r_t.task.x1 == elem.end.first && r_t.task.y1 == elem.end.second)
	{
	    ass.path_tot.path_a = CopiaPath(elem.wpoints);
	    break;
	}
    }
	
    //ora scrivo path_b prendendo dalla mappa la lista di waypoint che vanno dalla posizione di task_a all posizione di task_b di r_t
    for(auto elem : Map)
    {
	if(r_t.task.x1 == elem.start.first && r_t.task.y1 == elem.start.second && r_t.task.x2 == elem.end.first && r_t.task.y2 == elem.end.second)
	{
	    ass.path_tot.path_b = CopiaPath(elem.wpoints);	    
	    break;
	}
    }
    
    
    return ass;
}



// Legge "assignment_topic" 
void RTCallback(const task_assign::rt_vect::ConstPtr& msg)
{
    bool add(true);
    new_assign = false;
    struct Assign ass;
    
    if(msg->rt_vect.size() > 0)
    {	
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
		new_assign = true;
		
		robots_in_execution.push_back(rt.robot);
		tasks_in_execution.push_back(rt.task);
		
		ass = MapToCatal(GlobMap, rt);
		assignments_vect.push_back(ass.path_tot);
		
		// Crea il Catalogo_Ass
		Catalogo_Ass.push_back(ass);
	    }
	    
	    add = true;
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



// Pubblica al master su "rob_assign_topic" il vettore dei robot da assegnare
void publishRobotToAssign()
{
    task_assign::vect_robot vect_msg;
    
    vect_msg.robot_vect = available_robots;
    
    sleep(1);
    rob_ass_pub.publish(vect_msg);
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



// Pubblica al task manager i task completati su "task_exec_topic"
void publishRecharge()
{
    task_assign::rech_vect msg;
    
    double dist(0);
    double min_dist(1000);
    task_assign::recharge min_re;
    
    for(auto rob : robots_in_recharge)
    {
	for(auto re : Recharge)
	{
	    for(auto elem : GlobMap)
	    {
		if(rob.x == elem.start.first && rob.y == elem.start.second && re.x == elem.end.first && re.y == elem.end.second)
		{
		    dist = CalcPath(elem.wpoints);
		    break;
		}
	    }
	    
	    // vedo qual'è la distanza minima
	    if(dist < min_dist)
	    {
		min_dist = dist;
		re.r_name = rob.name;
		min_re = re;
	    }
	}
	
	// associo al robot il punto di carica a distanza minima
	msg.vector.push_back(min_re);
    }
    
    
    sleep(1);
    recharge_pub.publish(msg);
}





// Legge "obstacles_topic" 
void ObsCallback(const task_assign::vect_task::ConstPtr& msg)
{

}









// SearchRecharge(robots_in_recharge, Recharge, GlobMap);

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
    recharge_pub = node.advertise<task_assign::rech_vect>("recharge_topic", 10);
    
    sleep(1);

    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	ros::spinOnce();
	rate.sleep();
    }



    return 0;
}
