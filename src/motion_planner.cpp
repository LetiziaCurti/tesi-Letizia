#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <math.h>
#include "ros/ros.h"
#include "lemon_ros/geometry_tools.h"
#include "lemon/smart_graph.h"
#include "lemon/list_graph.h"
#include <lemon/lgf_reader.h>
#include <lemon/graph_to_eps.h>
#include <lemon/dim2.h>
#include <lemon/dijkstra.h>
#include <lemon/adaptors.h>
#include <lemon/concepts/maps.h>
#include <lemon/path.h>
#include <lemon/concepts/digraph.h>
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"
#include "task_assign/vect_robot.h"
#include "task_assign/rt_vect.h"
#include "task_assign/vect_info.h"
#include "task_assign/task_path.h"
#include "task_assign/assignment.h"
#include "task_assign/rech_vect.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "task_assign/glpk_in.h"
#include "yaml-cpp/yaml.h"


inline const char * const BoolToString(bool b)
{
    return b ? "true" : "false";
}

double getDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

using namespace std;
using namespace lemon;

ros::Subscriber obs_sub;
ros::Subscriber rt_sub;
ros::Subscriber rech_sub;
ros::Subscriber status_rob_sub;
ros::Subscriber new_task_sub;

ros::Publisher exec_task_pub;
ros::Publisher assignment_pub;
ros::Publisher recharge_pub;
ros::Publisher marker_pub;
ros::Publisher reass_pub;

#define VELOCITY 10
#define BATTERY_THR 10
#define SEC_DIST 1.5

bool new_assign(false);
bool new_in_rech(false);
bool new_task(false);
bool pub_master_in(false);


vector<task_assign::robot> available_robots;    	//R: vettore dei robot per l'assegnazione che cambia nel tempo (di dim n(k))
vector<task_assign::robot> robots_in_recharge;    	//vettore dei robot disponibili all'assegnazione di punti di ricarica
vector<task_assign::robot> robots_in_execution;    	//vettore dei robot che stanno eseguendo dei task
vector<task_assign::robot> robots_in_exec_rech;    	//vettore dei robot che stanno andando a ricaricarsi

vector<task_assign::task> tasks_to_assign;    		//T: vettore dei task da assegnare che cambia nel tempo di dimensione m(k)
vector<task_assign::task> tasks_in_execution;		//vettore dei task già in esecuzione
vector<task_assign::task> completed_tasks;		//vettore dei task completati che viene inviato al task_manager

vector<task_assign::task_path> assignments_vect;	//vettore delle info da inviare ai robots (nome del task e percorso per raggiungerlo)
vector<task_assign::task_path> robRech_vect;		//vettore degli assignments robot-punto di ricarica

vector<task_assign::info> rt_info_vect;
vector<task_assign::info> rech_info_vect;


//TODO carica il vettore recharge_points iniziale da un file yaml
vector<task_assign::task> recharge_points;   		// è la lista di tutti i punti di ricarica LIBERI presenti nello scenario
vector<task_assign::task> recharge_points_busy; 



//simulo una mappa
struct mappa
{
    pair<double,double> start;
    pair<double,double> end;
    vector<pair<double,double>> wpoints;
};

vector<mappa> GlobMap;   				// la mappa globale è un vettore di strutture mappa, ognuna rappresenta il path 
							// per andare da un robot ad un task, va passata dall'esterno

SmartDigraph Mappa; 
SmartDigraph::NodeMap<float> coord_x(Mappa);
SmartDigraph::NodeMap<float> coord_y(Mappa);
SmartDigraph::NodeMap<int> id(Mappa);
SmartDigraph::NodeMap<dim2::Point<float> > coords(Mappa);
SmartDigraph::ArcMap<double> len(Mappa);
map<int, SmartDigraph::Node> excl_task_nodes;


struct Assign
{
    task_assign::robot rob;
    task_assign::task task;
    task_assign::task_path path_tot;
};

vector<Assign> Catalogo_Ass;				//struttura che tiene in memoria tutti gli assignment task - robot
vector<Assign> Catalogo_Rech;				//struttura che tiene in memoria tutti gli assignment robot - p.to di ric.






void delNode(map<int,SmartDigraph::Node> excl_nodes)
{
    for(auto elem : excl_nodes)
    {
	for (SmartDigraph::OutArcIt a(Mappa, elem.second); a != INVALID; ++a)
	{
	    len[a]+=10000;
	}
	
	for (SmartDigraph::InArcIt a(Mappa, elem.second); a != INVALID; ++a)
	{
	    len[a]+=10000;
	}
    }
}



void insertNode(map<int,SmartDigraph::Node> excl_nodes)
{
    for(auto elem : excl_nodes)
    {
	for (SmartDigraph::OutArcIt a(Mappa, elem.second); a != INVALID; ++a)
	{
	    len[a]-=10000;
	}
	
	for (SmartDigraph::InArcIt a(Mappa, elem.second); a != INVALID; ++a)
	{
	    len[a]-=10000;
	}
    }
}






//TODO obstacle_node manda al motion planner gli id dei nodi che sono diventati ostacoli --> in corrispondenza
//dei "nodi ostacolo", vanno settati i pesi degli archi incidenti ad un numero elevatissimo

// Legge "obstacles_topic" 
void ObsCallback(const task_assign::vect_task::ConstPtr& msg)
{

}



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





// // TODO se non si vuole dare dall'esterno i task e i robot in base agli id ma in base alle posizioni, 
// // con una function bisogna cercare nel grafo il nodo n con le coords[n] uguali alla posizione, e prendere l'id del nodo n
// // ma computazionalmente è costosissimo quindi per ora si evita
// int poseToId(float x, float y, SmartDigraph maps)
// {
//     
// }



// // TODO Se si vuole aggiungere la possibilità che il robot sia in una posizione che non sta nella mappa, 
// // usa questa function per cercare il nodo più vicino e aggiungere nuovi nodo e arco alla mappa
// void addNodeToMap(double rx, double ry, double tx, double ty, SmartDigraph maps)
// {
//     double min_dist(1000);
//     double dist(0.0);
// }




// Function che calcola il tempo necessario a ciascun robot per raggiungere tutti i task
// se i=0 calcolo i percorsi nell'istante "iniziale", se i=1 calcolo t_ex negli altri istanti
vector<task_assign::info> CalcTex(vector<task_assign::info> info_vect, vector<task_assign::robot> robots, vector<task_assign::task> tasks, SmartDigraph& maps, int op)
{
    // vettore delle info in uscita
//     vector<task_assign::info> tex;
    task_assign::info info;
		
	
    // creo i vettori dei nodi corrispondenti alle posizioni dei robots e dei tasks
    vector<SmartDigraph::Node> rob_start_nodes;
    vector<SmartDigraph::Node> taska_goal_nodes;
    vector<SmartDigraph::Node> taskb_goal_nodes;
    
    // metti nei vettori i nodi corrispondenti
    for(auto elem : robots)
    {
	rob_start_nodes.push_back(SmartDigraph::nodeFromId(elem.id));
    }
    
    for(auto elem : tasks)
    {
	taska_goal_nodes.push_back(SmartDigraph::nodeFromId(elem.id1));
	if(elem.id1 == elem.id2) //il task è un rech. point
	    taskb_goal_nodes.push_back(SmartDigraph::nodeFromId(elem.id1));
	else
	    taskb_goal_nodes.push_back(SmartDigraph::nodeFromId(elem.id2));
    }
  

    
    
    //con l'alg. di Dijkstra trovo il percorso minimo che c'è tra i nodi che mi servono (quelli in corrispondenza delle 
    //posizioni di robot e task). il percorso consiste in una sequenza di nodi che viene memorizzata in un vettore, che 
    //poi viene ribaltato per avere la sequenza di wp che il robot deve percorrere per arrivare al task0
//     SmartDigraph::ArcMap<double> len(Mappa);
//     SmartDigraph::NodeMap<float> coord_x(Mappa);
//     SmartDigraph::NodeMap<float> coord_y(Mappa);
    vector<task_assign::waypoint> path;
    vector<SmartDigraph::Node> path_node;
    SmartDigraph::Arc arc;
    double time_a(0);
    double time_b(0);
    
    map<int, SmartDigraph::Node>::iterator it;
    SmartDigraph::Node n;
 
    
    
   
    
    for(int i=0; i<robots.size(); i++)
    {
	for(int j=0; j<tasks.size(); j++)
	{
	    info.r_name = robots[i].name;
	    info.t_name = tasks[j].name;
	    double dist(0);
	    
	    
	    // prima parte del task	    
	    
	    it=excl_task_nodes.find(tasks[j].id1);
	    excl_task_nodes.erase(it);
	    if(tasks[j].id1 != tasks[j].id2)
	    {
		it=excl_task_nodes.find(tasks[j].id2);
		excl_task_nodes.erase(it);
	    }
	    delNode(excl_task_nodes);
	    
	    Dijkstra<SmartDigraph, SmartDigraph::ArcMap<double>> dijkstra_test(Mappa,len);
	    
	    dijkstra_test.run(rob_start_nodes.at(i), taska_goal_nodes.at(j));
	    // se il task non coincide col robot
	    if (dijkstra_test.dist(taska_goal_nodes.at(j)) > 0)
	    { 
		// ad ogni iterazione, andando a ritroso l'alg. trovo il nodo precedente da cui è minimo il costo per 
		// arrivare al successivo
		// memorizzo la posizione del nodo in tmp e la metto nel vettore path, e metto il nodo nel vettore path_node
		// i vettori path e path_node sono vettori temporanei che mi servono per calcolare time_a e time_b
		for (SmartDigraph::Node v = taska_goal_nodes.at(j); v != rob_start_nodes.at(i); v = dijkstra_test.predNode(v))
		{
		    task_assign::waypoint tmp;
		    tmp.y = coord_y[v];
		    tmp.x = coord_x[v];
		    path.push_back(tmp);
		    path_node.push_back(v);
		}
		reverse(path.begin(),path.end());
		reverse(path_node.begin(),path_node.end());
		
		// il tempo di esecuzione è la somma dei pesi di tutti gli archi del path trovato		
		for(int k=0; k<path_node.size()-1; k++)
		{
		    arc = lemon::findArc(Mappa,path_node[k],path_node[k+1]);
		    dist += 1/VELOCITY*len[arc];
		}		
	    }
	    else
	    {
		time_a = 0;
	    }
	    
	    path.clear();
	    path_node.clear();
	    
	    
	    
	    // seconda parte del task
	    
	    dist = 0;
	    dijkstra_test.run(taska_goal_nodes.at(j), taskb_goal_nodes.at(j));
	    // se il task non coincide col robot
	    if (dijkstra_test.dist(taskb_goal_nodes.at(j)) > 0)
	    { 
		// ad ogni iterazione, andando a ritroso l'alg. trovo il nodo precedente da cui è minimo il costo per 
		// arrivare al successivo
		// memorizzo la posizione del nodo in tmp e la metto nel vettore path, e metto il nodo nel vettore path_node
		// i vettori path e path_node sono vettori temporanei che mi servono per calcolare time_a e time_b
		for (SmartDigraph::Node v = taskb_goal_nodes.at(j); v != taska_goal_nodes.at(j); v = dijkstra_test.predNode(v))
		{
		    task_assign::waypoint tmp;
		    tmp.y = coord_y[v];
		    tmp.x = coord_x[v];
		    path.push_back(tmp);
		    path_node.push_back(v);
		}
		reverse(path.begin(),path.end());
		reverse(path_node.begin(),path_node.end());
		
		// il tempo di esecuzione è la somma dei pesi di tutti gli archi del path trovato
		for(int k=0; k<path_node.size()-1; k++)
		{
		    arc = lemon::findArc(Mappa,path_node[k],path_node[k+1]);
		    dist += 1/VELOCITY*len[arc];
		}		
	    }
	    else
	    {
		time_b = 0;
	    }

	    // if op=0 carico le info in tex0
	    if(!op)
		info.t_ex0 = time_a + tasks[j].wait1 + time_b + tasks[j].wait2;
	    // else carico le info in tex
	    else
		info.t_ex = time_a + tasks[j].wait1 + time_b + tasks[j].wait2;
	    
	    bool in = false;
	    for(auto elem : info_vect)
	    {
		if(elem.r_name == robots[i].name && elem.t_name == tasks[j].name)
		{
		      elem = info;
		      in = true;
		      break;
		}
	    }
	    if(!in)
		info_vect.push_back(info);
	    
	    in = false;
	    
// 	    tex.push_back(info);
	    
	    insertNode(excl_task_nodes);
	    n = SmartDigraph::nodeFromId(tasks[j].id1);
	    excl_task_nodes[tasks[j].id1] = n;
	    n = SmartDigraph::nodeFromId(tasks[j].id2);
	    excl_task_nodes[tasks[j].id2] = n;
	}
    }
    
    return info_vect;
}



// Legge su "new_task_topic" il vettore dei task da eseguire T
void TaskToAssCallback(const task_assign::vect_task::ConstPtr& msg)
{
    bool new_task(true);
    SmartDigraph::Node n;
    
    for(auto elem : msg->task_vect)
    {
	// vedo se elem sta già in task_to_assign
	for(auto newel : tasks_to_assign)
	{
	    if(newel.id1 == elem.id1 && newel.id2 == elem.id2)
	    {
		new_task = false;
		break;
	    }
	}
	for(auto newel : tasks_in_execution)
	{
	    if(newel.id1 == elem.id1 && newel.id2 == elem.id2)
	    {
		new_task = false;
		break;
	    }
	}
	
	if(new_task)
	{
	    tasks_to_assign.push_back(elem);
	    ROS_INFO_STREAM("The motion_planner is storing the task to assign: "<< elem.name);
	    // metti nel vettore excl_task_nodes i nodi corrispondenti ai nuovi task
	    n = SmartDigraph::nodeFromId(elem.id1);
	    excl_task_nodes[elem.id1] = n;
	    n = SmartDigraph::nodeFromId(elem.id2);
	    excl_task_nodes[elem.id2] = n;
	}
	
	new_task = true;
    }
}



// Pubblica al task manager i task completati su "task_exec_topic"
void publishExecTask()
{
    task_assign::vect_task msg;
    
    msg.task_vect = completed_tasks;
    
    sleep(1);
    exec_task_pub.publish(msg);
    
    for(auto elem : completed_tasks)
    {
	ROS_INFO_STREAM("The motion_planner is publishing to the task_manager the completed task: "<< elem.name);
    }
}



void publishMasterIn()
{
    task_assign::glpk_in msg;
    
    msg.reassign = true;
    msg.rob_to_ass = available_robots;
    msg.task_to_ass = tasks_to_assign;
    msg.rob_in_rech = robots_in_recharge;
    msg.rob_info = rt_info_vect;
    msg.rech_rob_info = rech_info_vect;
    
    
    // Wait for the publisher to connect to subscribers
    sleep(1.0);
    reass_pub.publish(msg);

    ROS_INFO_STREAM("The motion_planner is publishing to the master: \n");
    ROS_INFO_STREAM("Robot disponibili: \n");
    for(auto elem : available_robots)
    {
	ROS_INFO_STREAM(elem.name << "\n");
    }
    ROS_INFO_STREAM("Task da assegnare: \n");
    for(auto elem : tasks_to_assign)
    {
	ROS_INFO_STREAM(elem.name << "\n");
    }
    ROS_INFO_STREAM("Robot da ricaricare: \n");
    for(auto elem : robots_in_recharge)
    {
	ROS_INFO_STREAM(elem.name << "\n");
    }
    ROS_INFO_STREAM("tempi di esec. robot-task: \n");
    for(auto elem : rt_info_vect)
    {
	ROS_INFO_STREAM("rob: " << elem.r_name << "task: " << elem.t_name);
    }
    ROS_INFO_STREAM("tempi di esec. robot-punto di ric.: \n");
    for(auto elem : rech_info_vect)
    {
	ROS_INFO_STREAM("rob: " << elem.r_name << "punto di ricarica: " << elem.t_name);
    }

}



void ReAssFunc(task_assign::robot msg, Assign ass)
{
    ROS_INFO_STREAM("Sto facendo reassignment");
    // se sono lontana da taska e non ho ancora eseguito taska
    if(!msg.taska && getDistance(msg.x, msg.y, ass.task.x1, ass.task.y1) > SEC_DIST)
    {
	robots_in_execution = deleteRob(msg.name, robots_in_execution);
	robots_in_recharge.push_back(msg);
	tasks_in_execution = deleteTask(ass.task.name, tasks_in_execution);
	tasks_to_assign.push_back(ass.task);
	Catalogo_Ass = deleteAss(msg.name, Catalogo_Ass);
	
	rt_info_vect = CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
	rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
	
// 	publishMasterIn();
	pub_master_in = true;
    }
    // se ho eseguito taska e ora sono lontana sia da taska sia da taskb
    // TODO intervieni con il modulo di salvataggio 
    else if(msg.taska && getDistance(msg.x, msg.y, ass.task.x2, ass.task.y2) > SEC_DIST)
    {
// 	robots_in_execution = deleteRob(msg->name, robots_in_execution);
// 	robots_in_recharge.push_back(*msg);
// 	tasks_in_execution = deleteTask(ass.task.name, tasks_in_execution);
// 	tasks_to_assign.push_back(ass.task);
// 	Catalogo_Ass = deleteAss(msg->name, Catalogo_Ass);
    } 			 
}


double tex0, tex;
task_assign::info i_rob;

// Legge "status_rob_topic" 
void StatusCallback(const task_assign::robot::ConstPtr& msg)
{    
    bool in_charge(false);
    bool in_execution(false);
    bool available(false);
    bool avail_rech(false);
    map<int, SmartDigraph::Node>::iterator it;
    
    if(msg->status)
    {	
	ROS_INFO_STREAM("il motion planner sta ascoltando il robot " << msg->name);
	
	// vedo se il robot è già in uno dei vettori
	for(auto elem : available_robots)
	{
	    if(elem.name == msg->name)
	    {
		ROS_INFO_STREAM("il robot " << msg->name << " e' gia' in available_robots");
		available = true;
		break;
	    }
	}
	
	if(!available)
	{
	    // vedo se il robot è già in uno dei vettori
	    for(auto elem : robots_in_recharge)
	    {
		if(elem.name == msg->name)
		{
		    ROS_INFO_STREAM("il robot " << msg->name << " e' gia' tra i robot da ricaricare");
		    avail_rech = true;
		    break;
		}
	    }
	}
	
	if(!available && !avail_rech)
	{
	    for(auto elem : robots_in_execution)
	    {
		if(elem.name == msg->name)
		{
		    ROS_INFO_STREAM("il robot " << msg->name << " e' gia' tra i robot in esecuzione");
		    in_execution = true;
		    break;
		}
	    }
	}
	
	if(!available && !avail_rech && !in_execution)
	{
	    for(auto elem : robots_in_exec_rech)
	    {
		if(elem.name == msg->name)
		{
		    ROS_INFO_STREAM("il robot " << msg->name << " e' gia' in ricarica");
		    in_charge = true;
		    break;
		}
	    }
	}
	
   
	if(!available && !avail_rech && !in_execution && !in_charge && msg->b_level > BATTERY_THR)
	{
	    available_robots.push_back(*msg);
	    ROS_INFO_STREAM("il robot " << msg->name << " viene messo in available_robots");
// 	    CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
// 	    
// 	    if(tasks_to_assign.size()>0)
// 	    {
// 		publishMasterIn();
// 	    }
	}
	
	else if(!available && !avail_rech && !in_execution && !in_charge && msg->b_level <= BATTERY_THR)
	{
	    robots_in_recharge.push_back(*msg);
	    ROS_INFO_STREAM("il robot " << msg->name << " viene messo in robots_in_recharge");
// 	    CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
// 	    
// 	    if(recharge_points.size()>0)
// 	    {
// 		publishMasterIn();
// 	    }
	}
	
	else if(available)
	{	    	    
	    if(msg->b_level <= BATTERY_THR)
	    {
		    robots_in_recharge.push_back(*msg);
		    rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
		    available_robots = deleteRob(msg->name, available_robots);
	    }
	    else
		rt_info_vect = CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);	      
	}
	
	else if(avail_rech)
	{	    	    
	   rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);      
	}
	
	else if(in_execution)
	{
	    ROS_INFO_STREAM("il robot " << msg->name << " è in posizione "<<floor(msg->x+0.5)<<" - "<<floor(msg->y+0.5));
	    ROS_INFO_STREAM("il robot " << msg->name << " ha livello di batteria "<<msg->b_level);
	    // cerco il task corrispondente al robot
	    for(auto ass : Catalogo_Ass)
	    {
		if(msg->name == ass.rob.name)
		{
		    // vedo se il robot è arrivato al task
		    if(floor(msg->x+0.5) == ass.task.x2 && floor(msg->y+0.5) == ass.task.y2)
		    {
			ROS_INFO_STREAM("il task " << ass.task.name << " è stato completato");
			
			completed_tasks.push_back(ass.task);
			publishExecTask();
			// aggiorno la mappa dei nodi esclusi
			it=excl_task_nodes.find(ass.task.id1);
			excl_task_nodes.erase(it);
			it=excl_task_nodes.find(ass.task.id2);
			excl_task_nodes.erase(it);
	    
			tasks_in_execution = deleteTask(ass.task.name, tasks_in_execution);
			robots_in_execution = deleteRob(msg->name, robots_in_execution);
			Catalogo_Ass = deleteAss(msg->name, Catalogo_Ass);
			
			if(msg->b_level > BATTERY_THR)
			{
			    available_robots.push_back(*msg);
			    rt_info_vect = CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
			    if(tasks_to_assign.size()>0)
			    {
// 				publishMasterIn();
				pub_master_in = true;
			    }
			}
			else
			{
			    robots_in_recharge.push_back(*msg);
			    rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
			    if(recharge_points.size()>0)
			    {
// 				publishMasterIn();
				pub_master_in = true;				
			    }			    
			}
		    }
		    // se il robot sta eseguendo il task e non ha ancora finito
		    else
		    {	    
			// verifica sul livello di batteria
			if(msg->b_level <= BATTERY_THR)
			{
			    // se la distanza dal task è inferiore alla soglia SEC_DIST, ce lo faccio arrivare e poi 
			    // lo manderò in carica
			    // (quindi non faccio nulla)
			  
			    //altrimenti faccio reassignment
			    ReAssFunc(*msg,ass);
			}
			// verifica sull'errore tra tempi di esecuzione				
			else
			{				
			    rt_info_vect = CalcTex(rt_info_vect, robots_in_execution, tasks_in_execution, Mappa, 1);
			    
			    //TODO cerca in rt_info_vect la coppia che sta in ass, mettila in i_rob e prendi tex0 e tex
			    tex0 = i_rob.t_ex0;
			    tex = i_rob.t_ex;
			    
			    if(tex-tex0 < 0 || tex-tex0 >= 1/tex0 + 1/ass.task.ar_time - 1/msg->b_level)
			    {
				ReAssFunc(*msg,ass);
			    }
			}
		    }
		    
		    break;		    
		}
	    }
	}
	
	else if(in_charge)
	{	    
	    // cerco il p.to di ric. corrispondente al robot
	    for(auto ass : Catalogo_Rech)
	    {
		if(msg->name == ass.rob.name)
		{
		    // vedo se il robot si è ricaricato
		    if(msg->b_level == msg->b_level0)
		    {
			robots_in_recharge = deleteRob(msg->name, robots_in_recharge);
			Catalogo_Rech = deleteAss(msg->name, Catalogo_Rech);
			
			available_robots.push_back(*msg);
			rt_info_vect = CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
			if(tasks_to_assign.size()>0)
			{
// 			    publishMasterIn();
			    pub_master_in = true;			  
			}
		    }
		    // se il robot sta andando nel punto di ricarica e non ha batteria, faccio reassignment
		    else if(!msg->taska)
		    {	    
			// verifica sul livello di batteria
			if(msg->b_level <= BATTERY_THR)
			{
			    // se la distanza dal task è inferiore alla soglia SEC_DIST, ce lo faccio arrivare e poi 
			    // lo manderò in carica
			    // (quindi non faccio nulla)
			  
			    //altrimenti faccio reassignment
			    ReAssFunc(*msg,ass);
			}
			// verifica sull'errore tra tempi di esecuzione				
			else
			{				
			    rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 1);
			    
			    //TODO cerca in rt_info_vect la coppia che sta in ass, mettila in i_rob e prendi tex0 e tex
			    tex0 = i_rob.t_ex0;
			    tex = i_rob.t_ex;
			    
			    if(tex-tex0 < 0 || tex-tex0 >= 1/tex0 + 1/ass.task.ar_time - 1/msg->b_level)
			    {
				ReAssFunc(*msg,ass);
			    }
			}
		    }
		    
		    break;		    
		}
	    }
	}
	
	bool as = false;
	if(tasks_to_assign.size()>0 && available_robots.size()>0)
	{
	    rt_info_vect = CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
	    as = true;
// 	    publishMasterIn();
	}
	if(recharge_points.size()>0 && robots_in_recharge.size()>0)
	{
	    rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
	    as = true;
// 	    publishMasterIn();
	}
	if(as)
	    pub_master_in = true;	  
// 	    publishMasterIn();
    }
    // se il robot è rotto
    else
    {
	// TODO modulo di salvataggio
    }
}




// Function che mette in un oggetto di tipo Assign (che verrà messo poi nel Catalogo_Ass) il robot e il task di un assignment 
// e il percorso che deve fare il robot per raggiungere il task (prima fino task_a e poi da task_a a task_b)
Assign MinPath(task_assign::rt r_t)
{
    Assign ass;   
//     SmartDigraph::ArcMap<double> len(Map);
//     SmartDigraph::NodeMap<float> coord_x(Map);
//     SmartDigraph::NodeMap<float> coord_y(Map);
    SmartDigraph::Node rob;
    SmartDigraph::Node taska;
    SmartDigraph::Node taskb;
    task_assign::waypoint tmp;
    vector<task_assign::waypoint> path;
    vector<SmartDigraph::Node> path_node;
    
    map<int, SmartDigraph::Node>::iterator it;
    SmartDigraph::Node n;
  
  
    ass.rob = r_t.robot;
    ass.task = r_t.task;
    
    ass.path_tot.r_name = r_t.robot.name;
    ass.path_tot.t_name = r_t.task.name;
    ass.path_tot.id_a = r_t.task.id1;
    ass.path_tot.id_b = r_t.task.id2;
    
    rob = SmartDigraph::nodeFromId(r_t.robot.id);
    taska = SmartDigraph::nodeFromId(r_t.task.id1);
    if(r_t.task.id1 == r_t.task.id2) //il task è un rech. point
	taskb = SmartDigraph::nodeFromId(r_t.task.id1);
    else
	taskb = SmartDigraph::nodeFromId(r_t.task.id2);
    

    // prima parte del task
    // ora scrivo path_a prendendo dalla mappa la lista di waypoint che vanno dalla posizione del robot alla posizione di 
    // task_a di r_t
    it=excl_task_nodes.find(r_t.task.id1);
    excl_task_nodes.erase(it);
    if(r_t.task.id1 != r_t.task.id2) 
    {
	it=excl_task_nodes.find(r_t.task.id2);
	excl_task_nodes.erase(it);
    }
    delNode(excl_task_nodes);
    
    Dijkstra<SmartDigraph, SmartDigraph::ArcMap<double>> dijkstra_test(Mappa,len);

    dijkstra_test.run(rob, taska);
    // se il task non coincide col robot
    if (dijkstra_test.dist(taska) > 0)
    { 
	// ad ogni iterazione, andando a ritroso l'alg. trovo il nodo precedente da cui è minimo il costo per 
	// arrivare al successivo
	// memorizzo la posizione del nodo in tmp e la metto nel vettore path, e metto il nodo nel vettore path_node
	// i vettori path e path_node sono vettori temporanei che mi servono per calcolare time_a e time_b
	for (SmartDigraph::Node v = taska; v != rob; v = dijkstra_test.predNode(v))
	{
	    tmp.y = coord_y[v];
	    tmp.x = coord_x[v];
	    path.push_back(tmp);
	    path_node.push_back(v);
	}
	reverse(path.begin(),path.end());
	reverse(path_node.begin(),path_node.end());	
    }
    else
    {
	tmp.x = r_t.robot.x;
	tmp.y = r_t.robot.y;
	path.push_back(tmp);
    }

    ass.path_tot.path_a = path;
    path.clear();
    path_node.clear();
       
    
    
    // seconda parte del task
    
    // rimetto gli archi entranti e uscenti di taskb a len originaria (tolgo 1000)
//     it=excl_task_nodes.find(r_t.task.id2);
//     excl_task_nodes.erase(it);
//     delNode(excl_task_nodes);
    
//     Dijkstra<SmartDigraph, SmartDigraph::ArcMap<double>> dijkstra_test2(Mappa,len);

    dijkstra_test.run(taska, taskb);
    // se il task non coincide col robot
    if (dijkstra_test.dist(taskb) > 0)
    { 
	// ad ogni iterazione, andando a ritroso l'alg. trovo il nodo precedente da cui è minimo il costo per 
	// arrivare al successivo
	// memorizzo la posizione del nodo in tmp e la metto nel vettore path, e metto il nodo nel vettore path_node
	// i vettori path e path_node sono vettori temporanei che mi servono per calcolare time_a e time_b
	for (SmartDigraph::Node v = taskb; v != taska; v = dijkstra_test.predNode(v))
	{
	    tmp.y = coord_y[v];
	    tmp.x = coord_x[v];
	    path.push_back(tmp);
	    path_node.push_back(v);
	}
	reverse(path.begin(),path.end());
	reverse(path_node.begin(),path_node.end());	
    }
    // altrimenti taska coincide con taskb, questo significa che il task è un punto di ricarica
    else
    {
	tmp.x = r_t.task.x1;
	tmp.y = r_t.task.y1;
	path.push_back(tmp);
    }
    
    ass.path_tot.path_b = path;
    path.clear();
    path_node.clear();
    
    
    insertNode(excl_task_nodes);
    excl_task_nodes[r_t.task.id1] = SmartDigraph::nodeFromId(r_t.task.id1);
    excl_task_nodes[r_t.task.id2] = SmartDigraph::nodeFromId(r_t.task.id2);
	    
    return ass;
}



// Legge "assignment_topic" 
void RTCallback(const task_assign::rt_vect::ConstPtr& msg)
{
    bool add(true);
    struct Assign ass;
    
    if(msg->rt_vect.size() > 0)
    {	
	//metto le nuove coppie r-t nel catalogo, gli associo il percorso selezionandolo dalla mappa globale, metto
	// il robot in robots_in_execution e il task in tasks_in_execution
	for(auto rt : msg->rt_vect)
	{
	    ROS_INFO_STREAM("il motion planner ha ricevuto dal master la coppia r-t "<< rt.robot.name << " - " << rt.task.name);
	    ROS_INFO_STREAM("il task ha coordinate:	taska "<< rt.task.x1 << " - " << rt.task.y1<<"	taskb: "<< rt.task.x2 << " - " << rt.task.y2);
		    
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
		
		
// 		ass = MapToCatal(GlobMap, rt, 1);
		ass = MinPath(rt);
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
    struct Assign ass;
    
    if(msg->rt_vect.size() > 0)
    {	
	//metto le nuove coppie r-t nel catalogo, gli associo il percorso selezionandolo dalla mappa globale, metto
	// il robot in robots_in_execution e il task in tasks_in_execution
	for(auto rt : msg->rt_vect)
	{
	    ROS_INFO_STREAM("il motion planner ha ricevuto dal master la coppia r-rech.point "<< rt.robot.name << " - " << rt.task.name);
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
		
		robots_in_exec_rech.push_back(rt.robot);
		robots_in_recharge = deleteRob(rt.robot.name, robots_in_recharge);
		
		recharge_points_busy.push_back(rt.task);
		recharge_points = deleteTask(rt.task.name, tasks_to_assign);
		
// 		ass = MapToCatal(GlobMap, rt, 0);
		ass = MinPath(rt);
		robRech_vect.push_back(ass.path_tot);
		
		// Crea il Catalogo_Ass
		Catalogo_Rech.push_back(ass);
	    }
	    
	    add = true;
	}
    }
}



// if(new_assign) ....pubblica
// Pubblica ai robot i task rispettivamente assegnati
void publishAssign()
{
    task_assign::assignment msg;
    
    msg.assign_vect = assignments_vect;
    
    sleep(1);
    assignment_pub.publish(msg);
    
    for(auto elem : msg.assign_vect)
    {
	ROS_INFO_STREAM("The motion_planner is publishing to the robot: " << elem.r_name << " the assigned task " << elem.t_name);
// 	for(auto wp : elem.path_a)
// 	{
// 	  ROS_INFO_STREAM("coordinate dei wp per taska: "<< wp.x <<" - " << wp.y);
// 	} 
    }
}



// Pubblica ai robot che devono andare in carica il loro punto di ricarica
void publishRecharge()
{
    task_assign::assignment msg;
    
    msg.assign_vect = robRech_vect;
    
    sleep(1);
    recharge_pub.publish(msg);
}



// i wp sono cubi verdi
void publishMarker(task_assign::waypoint p, int id_marker)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "motion_planner";
    marker.id = id_marker;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

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
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7;

    marker.lifetime = ros::Duration();

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



// Function con cui carico i punti di ricarico dal file yaml al vettore recharge_points
void RechPoints()
{
    task_assign::task newTask;
    
    // Leggi tutte le info da un file yaml e mettile al posto di new_task_vect
    YAML::Node node_conf = YAML::LoadFile("/home/letizia/catkin_ws/src/task_assign/config/rech_points_config.yaml");
    const YAML::Node& node_test1 = node_conf["RECH_POINTS"];

    for (std::size_t i = 0; i < node_test1.size(); i++) 
    {
	const YAML::Node& node_test2 = node_test1[i];
	newTask.name = node_test2["name"].as<std::string>();
// 	std::cout << "Name: " << node_test2["name"].as<std::string>() << std::endl;
	newTask.ar_time = node_test2["arrt"].as<double>();
// 	std::cout << "Arrival time: " << node_test2["arrt"].as<double>() << std::endl;
	
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
	    
// 	    std::cout << "Id"<< i+1 << ": " << node_test4["id"].as<double>() << std::endl;
// 	    std::cout << "Wait: " << node_test4["wait"].as<double>() << std::endl;
	    
	    const YAML::Node& node_pos = node_test4["position"];
	    for (std::size_t j = 0; j < node_pos.size(); j++) 
	    {
		if(i==0)
		{
		    if(j==0)
		    {
			newTask.x1 = node_pos[j].as<double>();
// 			std::cout << "x: " << node_pos[j].as<double>() << std::endl;
		    }
		    else  if(j==1)
		    {
			newTask.y1 = node_pos[j].as<double>();
// 			std::cout << "y: " << node_pos[j].as<double>() << std::endl;
		    }
		    else  if(j==2)
		    {
			newTask.theta1 = node_pos[j].as<double>();
// 			std::cout << "theta: " << node_pos[j].as<double>() << std::endl;
		    }
		}
		else if(i==1)
		{
		    if(j==0)
		    {
			newTask.x2 = node_pos[j].as<double>();
// 			std::cout << "x: " << node_pos[j].as<double>() << std::endl;
		    }
		    else  if(j==1)
		    {
			newTask.y2 = node_pos[j].as<double>();
// 			std::cout << "y: " << node_pos[j].as<double>() << std::endl;
		    }
		    else  if(j==2)
		    {
			newTask.theta2 = node_pos[j].as<double>();
// 			std::cout << "theta: " << node_pos[j].as<double>() << std::endl;
		    }
		}
	    }
	}
	
	recharge_points.push_back(newTask);	
    }
}








int main(int argc, char **argv)
{ 
    // Initialize the node
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle node;
    
    obs_sub = node.subscribe("obstacles_topic", 20, &ObsCallback);
    rt_sub = node.subscribe("rt_topic", 20, &RTCallback);
    rech_sub = node.subscribe("rech_topic", 20, &RechCallback);
    status_rob_sub = node.subscribe("status_rob_topic", 20, &StatusCallback);
    new_task_sub = node.subscribe("new_task_topic", 20, &TaskToAssCallback);
    
    exec_task_pub = node.advertise<task_assign::vect_task>("task_exec_topic", 10);   
    assignment_pub = node.advertise<task_assign::assignment>("assignment_topic", 10);
    recharge_pub = node.advertise<task_assign::assignment>("recharge_topic", 10);
    
    marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    reass_pub = node.advertise<task_assign::glpk_in>("glpk_in_topic", 10);
    
    
    sleep(1);

    
    // Carico i recharge points in recharge_points e poi metto i nodi del grafo corrispondenti in excl_task_nodes
    RechPoints();   
    for(auto elem : recharge_points)
    {
	excl_task_nodes[elem.id1] = SmartDigraph::nodeFromId(elem.id1);
    }
    
    
    // Carica il grafo dal file .lgf, e lo mette nel grafo orientato Mappa (var globale)
    try
    {
	digraphReader(Mappa, "/home/letizia/catkin_ws/src/task_assign/config/griglia.gml.lgf")
	.nodeMap("coordinates_x",coord_x)
	.nodeMap("coordinates_y",coord_y)
        .nodeMap("label",id)   
 	.run();
    }
    catch (Exception& error)
    {
	std::cerr << "Error: " << error.what() << std::endl;
	exit(1);
    }
    
    for (SmartDigraph::NodeIt n(Mappa); n != INVALID; ++n)
    {
	coord_x[n]=coord_x[n]/10;
	coord_y[n]=coord_y[n]/10;
	coords[n]=dim2::Point<float>(coord_x[n], coord_y[n]);
	
// 	task_assign::waypoint wp;
// 	wp.x = coord_x[n];
// 	wp.y = coord_y[n];
// 	publishMarker(wp,id[n]);
    }

    for (SmartDigraph::ArcIt a(Mappa); a != INVALID; ++a)
    {
	double dist = getDistance(coord_x[Mappa.source(a)],coord_y[Mappa.source(a)],coord_x[Mappa.target(a)],coord_y[Mappa.target(a)]);
	len[a] = dist;
    }
    

    

    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	ros::spinOnce();
	while(!pub_master_in && !new_assign && !new_in_rech && ros::ok())
	{
	    publishAssign();
	    ros::spinOnce();
	    rate.sleep();	  
	}
	if(pub_master_in && ros::ok())
	{
	    sleep(1);
	    publishMasterIn();
	    pub_master_in = false;
	}
	if(new_assign && ros::ok())
	{
// 	    sleep(1);
	    publishAssign();
	    new_assign = false;
	}
	if(new_in_rech && ros::ok())
	{
	    publishRecharge();
	    new_in_rech = false;
	}	    

	ros::spinOnce();
	rate.sleep();
    }		



    return 0;
}
