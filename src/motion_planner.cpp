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
ros::Subscriber rech_sub;
ros::Subscriber status_rob_sub;
ros::Subscriber new_task_sub;

ros::Publisher exec_task_pub;
ros::Publisher rob_ass_pub;
ros::Publisher rob_rech_pub;
ros::Publisher task_ass_pub;
ros::Publisher rob_ini_pub;
ros::Publisher rob_info_pub;
ros::Publisher rech_info_pub;
ros::Publisher assignment_pub;
ros::Publisher recharge_pub;

#define VELOCITY 10
#define BATTERY_THR 10
#define SEC_DIST 1.5

bool new_assign(false);
bool new_in_rech(false);
bool new_task(false);


vector<task_assign::robot> available_robots;    	//R: vettore dei robot per l'assegnazione che cambia nel tempo (di dim n(k))
vector<task_assign::robot> robots_in_execution;    	//vettore dei robot che stanno eseguendo dei task o che stanno andando a ricaricarsi
vector<task_assign::robot> robots_in_recharge;    	//vettore dei robot che stanno eseguendo dei task o che stanno andando a ricaricarsi
vector<task_assign::task> tasks_to_assign;    		//T: vettore dei task da assegnare che cambia nel tempo di dimensione m(k)
vector<task_assign::task> tasks_in_execution;		//vettore dei task già in esecuzione
vector<task_assign::task> completed_tasks;		//vettore dei task completati che viene inviato al task_manager
vector<task_assign::task_path> assignments_vect;	//vettore delle info da inviare ai robots (nome del task e percorso per raggiungerlo)
vector<task_assign::task_path> robRech_vect;		//vettore degli assignments robot-punto di ricarica
vector<task_assign::info> rob_info_vect;
vector<task_assign::info> rob_info0_vect;
vector<task_assign::info> rech_info_vect;



//simulo una mappa
struct mappa
{
    pair<double,double> start;
    pair<double,double> end;
    vector<pair<double,double>> wpoints;
};

vector<mappa> GlobMap;   				// la mappa globale è un vettore di strutture mappa, ognuna rappresenta il path 
							// per andare da un robot ad un task, va passata dall'esterno
vector<task_assign::task> recharge_points;   		// è la lista di tutti i punti di ricarica presenti nello scenario, va passata dall'esterno



struct Assign
{
    task_assign::robot rob;
    task_assign::task task;
    task_assign::task_path path_tot;
};

vector<Assign> Catalogo_Ass;				//struttura che tiene in memoria tutti gli assignment task - robot
vector<Assign> Catalogo_Rech;				//struttura che tiene in memoria tutti gli assignment robot - p.to di ric.






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



// Function che elimina il task con il nome passato in argomento dal vettore di task passato in argomento
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



// Function che elimina dal catalogo l'assignment del robot con il nome passato in argomento
vector<Assign> deleteAss(string name, vector<Assign> vect)
{
    int i(0);
    for(auto elem : vect)
    {
	if(elem.rob.name == name)
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



// Function che calcola il path partendo dalla posizione di un robot che non sta tra i wp della mappa
// idea: cerca il wp più vicino sulla mappa e somma la distanza fino al wp con il resto del path (dal wp al task)
vector<pair<double,double>> CalcDistMap(double rx, double ry, double tx, double ty, vector<mappa> maps)
{
    double min_dist(1000);
    double dist(0.0);
    pair<double,double> coord;
    coord = make_pair(rx, ry);
    mappa min_elem;

    
    for(auto elem : maps)
    {
    	dist = getDistance(rx, ry, elem.start.first, elem.start.second);
	if(dist <= SEC_DIST && tx == elem.end.first && ty == elem.end.second)
	{
	    if(dist < min_dist)
	    {
	    	min_dist = dist;
	    	min_elem.start = elem.start;
	    	min_elem.end = elem.end;
	    	min_elem.wpoints = elem.wpoints;
	    }
	}
    }
    
    min_elem.wpoints.insert(min_elem.wpoints.begin(), coord);
    min_elem.start = coord;
    GlobMap.push_back(min_elem);
    
    return min_elem.wpoints;
}



// Function che calcola il tempo necessario a ciascun robot per raggiungere tutti i task
// se i=0 calcolo i percorsi nell'istante "iniziale", se i=1 calcolo t_ex negli altri istanti
vector<task_assign::info> CalcTex(vector<task_assign::robot> robots, vector<task_assign::task> tasks, vector<mappa> maps, int i)
{
    vector<task_assign::info> tex;
    task_assign::info info;
    double time_a(0);
    double time_b(0);
    bool in_map(false);
    
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
		    in_map = true;
		    time_a = 1/VELOCITY*CalcPath(elem.wpoints);
		    break;
		}
	    }
	    if(!in_map)
	    {
		time_a = 1/VELOCITY*CalcPath(CalcDistMap(rob.x, rob.y, task.x1, task.y1, GlobMap));
	    }
	    in_map = false;
	    
	    for(auto elem : maps)
	    {
		if(task.x1 == elem.start.first && task.y1 == elem.start.second && task.x2 == elem.end.first && task.y2 == elem.end.second)
		{
		    time_b = 1/VELOCITY*CalcPath(elem.wpoints);
		    break;
		}
	    }
	    
	    if(!i)
	    {
		info.t_ex0 = time_a + task.wait1 + time_b + task.wait1;
		info.t_ex = time_a + task.wait1 + time_b + task.wait1;
	    }
	    else
		info.t_ex = time_a + task.wait1 + time_b + task.wait1;
	    
	    tex.push_back(info);
	}
    }
    
    return tex;
}



// Legge su "new_task_topic" il vettore dei task da eseguire T
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
	{
	    tasks_to_assign.push_back(elem);
	    new_task = true;
	}
	
	add_task = true;
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
			Catalogo_Ass = deleteAss(msg->name, Catalogo_Ass);
			
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
				  robots_in_execution = deleteRob(msg->name, robots_in_execution);
				  robots_in_recharge.push_back(*msg);
				  tasks_in_execution = deleteTask(ass.task.name, tasks_in_execution);
				  tasks_to_assign.push_back(ass.task);
				  Catalogo_Ass = deleteAss(msg->name, Catalogo_Ass);
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
	    {
		robots_in_recharge = deleteRob(msg->name, robots_in_recharge);
		Catalogo_Rech = deleteAss(msg->name, Catalogo_Rech);
		available_robots.push_back(*msg);
	    }
	}
    }
    
    //vettore dei tempi di esecuzione di ciascun robot rispetto a tutti i task
    rob_info0_vect = CalcTex(available_robots, tasks_to_assign, GlobMap, 0);
    rob_info_vect = CalcTex(robots_in_execution, tasks_to_assign, GlobMap, 1);
    
    // vettore dei tempi di esecuzione di ciascun robot rispetto a tutti i punti ri ricarica
    rech_info_vect = CalcTex(robots_in_recharge, recharge_points, GlobMap, 1); 
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
Assign MapToCatal(vector<mappa> Map, task_assign::rt r_t, int i)
{
    vector<task_assign::waypoint> path;
    task_assign::waypoint wp;
    Assign ass;
    bool in_map(false);
  
  
    ass.rob = r_t.robot;
    ass.task = r_t.task;
    
    ass.path_tot.r_name = r_t.robot.name;
    ass.path_tot.t_name = r_t.task.name;
    ass.path_tot.id_a = r_t.task.id1;
    ass.path_tot.id_b = r_t.task.id2;
    
    //ora scrivo path_a prendendo dalla mappa la lista di waypoint che vanno dalla posizione del robot alla posizione di task_a di r_t
    for(auto elem : Map)
    {
	if(r_t.robot.x == elem.start.first && r_t.robot.y == elem.start.second && r_t.task.x1 == elem.end.first && r_t.task.y1 == elem.end.second)
	{
	    in_map = true;
	    ass.path_tot.path_a = CopiaPath(elem.wpoints);
	    break;
	}
    }
    if(!in_map)
    {
	ass.path_tot.path_a = CopiaPath(CalcDistMap(r_t.robot.x, r_t.robot.y, r_t.task.x1, r_t.task.y1, GlobMap));
    }

    //se i=1 sto usando la function per gli assignment, altrimenti per i recharge e quindi non serbe task b
    if(i)
    {
	//ora scrivo path_b prendendo dalla mappa la lista di waypoint che vanno dalla posizione di task_a all posizione di task_b di r_t
	for(auto elem : Map)
	{
	    if(r_t.task.x1 == elem.start.first && r_t.task.y1 == elem.start.second && r_t.task.x2 == elem.end.first && r_t.task.y2 == elem.end.second)
	    {
		ass.path_tot.path_b = CopiaPath(elem.wpoints);	    
		break;
	    }
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
		available_robots = deleteRob(rt.robot.name, available_robots);
		
		tasks_in_execution.push_back(rt.task);
		tasks_to_assign = deleteTask(rt.task.name, tasks_to_assign);
		
		
		ass = MapToCatal(GlobMap, rt, 1);
		assignments_vect.push_back(ass.path_tot);
		
		// Crea il Catalogo_Ass
		Catalogo_Ass.push_back(ass);
	    }
	    
	    add = true;
	}
    }
}



// Legge "recharge_topic" 
void RechCallback(const task_assign::rt_vect::ConstPtr& msg)
{
    bool add(true);
    new_in_rech = false;
    struct Assign ass;
    
    if(msg->rt_vect.size() > 0)
    {	
	//metto le nuove coppie r-t nel catalogo, gli associo il percorso selezionandolo dalla mappa globale, metto
	// il robot in robots_in_execution e il task in tasks_in_execution
	for(auto rt : msg->rt_vect)
	{
	    //vedo se è già nel catalogo
	    for(auto elem : Catalogo_Rech)
	    {
		if(elem.rob.name==rt.robot.name && elem.task.name==rt.task.name)
		{
		    add = false;
		    break;
		}
	    }
	    
	    if(add)
	    {
		new_in_rech = true;
		
		ass = MapToCatal(GlobMap, rt, 0);
		robRech_vect.push_back(ass.path_tot);
		
		// Crea il Catalogo_Ass
		Catalogo_Rech.push_back(ass);
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



// Pubblica al master il vettore dei task da eseguire task_to_assign
void publishTaskToAssign()
{
    task_assign::vect_task vect_msg; 
    
    vect_msg.task_vect = tasks_to_assign;

    // Wait for the publisher to connect to subscribers
    sleep(1.0);
    task_ass_pub.publish(vect_msg);
    
//     for(auto elem : vect_msg.task_vect)
//     {
// 	ROS_INFO_STREAM("The task_manager is publishing the task to assign: "<< elem.name << " whit the couple " << elem.name1 << " - " << elem.name2);
//     }
}



// Pubblica al master su "rob_assign_topic" il vettore dei robot da assegnare
void publishRobotToAssign()
{
    task_assign::vect_robot vect_msg;
    
    vect_msg.robot_vect = available_robots;
    
    sleep(1);
    rob_ass_pub.publish(vect_msg);
}



// Pubblica al master su "rob_assign_topic" il vettore dei robot da assegnare
void publishRobInRecharge()
{
    task_assign::vect_robot vect_msg;
    
    vect_msg.robot_vect = robots_in_recharge;
    
    sleep(1);
    rob_ass_pub.publish(vect_msg);
}



// Pubblica al master su "rob_info_topic" le info relative ai robot (tutti: già assegnati e da assegnare), pubblica tex0_info_vect
void publishRobotInfo()
{
    task_assign::vect_info vect_msg;
    
    vect_msg.info_vect = rob_info_vect;
    vect_msg.info0_vect = rob_info0_vect;
    
    sleep(1);
    rob_info_pub.publish(vect_msg);
   
}



// Pubblica al master i tempi richiesti da ogni ogni robot per raggiungere ciascun punto di ricarica
void publishInfoRecharge()
{
    task_assign::vect_info vect_msg;
    
    vect_msg.info_vect = rech_info_vect;
//     vect_msg.info0_vect = rech_info0_vect;
    
    sleep(1);
    rech_info_pub.publish(vect_msg);
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



// Pubblica ai robot che devono andare in carica il loro punto di ricarica
void publishRecharge()
{
    task_assign::assignment msg;
    
    msg.assign_vect = robRech_vect;
    
    sleep(1);
    recharge_pub.publish(msg);
}








// Legge "obstacles_topic" 
void ObsCallback(const task_assign::vect_task::ConstPtr& msg)
{

}










int main(int argc, char **argv)
{ 
    // Initialize the node
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle node;
    
    obs_sub = node.subscribe("obstacles_topic", 20, &ObsCallback);
    rt_sub = node.subscribe("rt_topic", 20, &RTCallback);
    rech_sub = node.subscribe("rech_topic", 20, &RechCallback);
//     arr_rob_sub = node.subscribe("status_rob_topic", 20, &ArrCallback);
    status_rob_sub = node.subscribe("status_rob_topic", 20, &StatusCallback);
    new_task_sub = node.subscribe("new_task_topic", 20, &TaskToAssCallback);
    
    exec_task_pub = node.advertise<task_assign::vect_task>("task_exec_topic", 10);   
    rob_ass_pub = node.advertise<task_assign::vect_robot>("rob_assign_topic", 10);
    rob_rech_pub = node.advertise<task_assign::vect_robot>("rob_recharge_topic", 10);
    task_ass_pub = node.advertise<task_assign::vect_task>("task_assign_topic", 10);
//     rob_ini_pub = node.advertise<task_assign::vect_info>("rob_ini_topic", 10);
    rob_info_pub = node.advertise<task_assign::vect_info>("rob_info_topic", 10);
    rech_info_pub = node.advertise<task_assign::vect_info>("rech_info_topic", 10);
    assignment_pub = node.advertise<task_assign::assignment>("assignment_topic", 10);
    recharge_pub = node.advertise<task_assign::assignment>("recharge_topic", 10);
    
    sleep(1);

    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	while(!new_assign && ros::ok())
	{
	    if(available_robots.size() > 0 && tasks_to_assign.size() > 0)
	    {
		publishTaskToAssign();
		publishRobotToAssign();
		publishRobotInfo();
	    }
	    
	    if(completed_tasks.size() > 0)
		publishExecTask();
	    
	    if(robots_in_recharge.size() > 0)
		publishRecharge();	  
	}
      
	if(new_assign && ros::ok())
	{
	    publishAssign();
	    
	    if(completed_tasks.size() > 0)
		publishExecTask();
	    
	    if(robots_in_recharge.size() > 0)
		publishRecharge();
	    
	    new_assign = false;
	}
	
	
	if(available_robots.size() > 0 && tasks_to_assign.size() > 0)
	{
	    publishTaskToAssign();
	    publishRobotToAssign();
	    publishRobotInfo();
	}
	
	if(completed_tasks.size() > 0)
	    publishExecTask();
	
	if(robots_in_recharge.size() > 0)
	    publishRecharge();
	    

	ros::spinOnce();
	rate.sleep();
    }		



    return 0;
}
