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
// ros::Publisher rob_ass_pub;
// ros::Publisher rob_rech_pub;
// ros::Publisher task_ass_pub;
// ros::Publisher rob_info_pub;
// ros::Publisher rech_info_pub;
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
// vector<task_assign::info> rob_info0_vect;

vector<task_assign::info> rech_info_vect;
// vector<task_assign::info> avail_info_vect;
// vector<task_assign::info> exec_info_vect;


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


struct Assign
{
    task_assign::robot rob;
    task_assign::task task;
    task_assign::task_path path_tot;
};

vector<Assign> Catalogo_Ass;				//struttura che tiene in memoria tutti gli assignment task - robot
vector<Assign> Catalogo_Rech;				//struttura che tiene in memoria tutti gli assignment robot - p.to di ric.





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
void CalcTex(vector<task_assign::info> info_vect, vector<task_assign::robot> robots, vector<task_assign::task> tasks, SmartDigraph& maps, int op)
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
	taskb_goal_nodes.push_back(SmartDigraph::nodeFromId(elem.id2));
    }
  

    
    
    //con l'alg. di Dijkstra trovo il percorso minimo che c'è tra i nodi che mi servono (quelli in corrispondenza delle 
    //posizioni di robot e task). il percorso consiste in una sequenza di nodi che viene memorizzata in un vettore, che 
    //poi viene ribaltato per avere la sequenza di wp che il robot deve percorrere per arrivare al task0
    SmartDigraph::ArcMap<double> len(maps);
    SmartDigraph::NodeMap<float> coord_x(maps);
    SmartDigraph::NodeMap<float> coord_y(maps);
    vector<task_assign::waypoint> path;
    vector<SmartDigraph::Node> path_node;
    SmartDigraph::Arc arc;
    double time_a(0);
    double time_b(0);
    double dist(0); 
    
    
    Dijkstra<SmartDigraph, SmartDigraph::ArcMap<double>> dijkstra_test(maps,len);
    
    for(int i=0; i<robots.size(); i++)
    {
	for(int j=0; j<tasks.size(); j++)
	{
	    info.r_name = robots[i].name;
	    info.t_name = tasks[j].name;
	    

	    // prima parte del task
	    
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
		for(int i=0; i<path_node.size()-1; i++)
		{
		    arc = lemon::findArc(maps,path_node[i],path_node[i+1]);
		    dist += 1/VELOCITY*len[arc];
		}
		
	    }
	    else
	    {
		time_a = 0.001;
	    }
	    
	    path.clear();
	    path_node.clear();
	    
	    
	    
	    // seconda parte del task
	    
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
		for(int i=0; i<path_node.size()-1; i++)
		{
		    arc = lemon::findArc(maps,path_node[i],path_node[i+1]);
		    dist += 1/VELOCITY*len[arc];
		}
		
	    }
	    else
	    {
		time_b = 0.001;
	    }

	    // if op=0 carico le info in tex0
	    if(!op)
		info.t_ex0 = time_a + tasks[j].wait1 + time_b + tasks[j].wait2;
	    // else carico le info in tex
	    else
		info.t_ex = time_a + tasks[j].wait1 + time_b + tasks[j].wait2;
	    
	    
	    for(auto elem : info_vect)
	    {
		if(elem.r_name == robots[i].name && elem.t_name == tasks[j].name)
		{
		      elem = info;
		      break;
		}
	    }
	    
// 	    tex.push_back(info);
	}
    }
    
//     return tex;
}



// Legge su "new_task_topic" il vettore dei task da eseguire T
void TaskToAssCallback(const task_assign::vect_task::ConstPtr& msg)
{
    bool new_task(true);
    
    for(auto elem : msg->task_vect)
    {
	// vedo se elem sta già in task_to_assign
	for(auto newel : tasks_to_assign)
	{
	    if(newel.id1 == elem.id1 && newel.id2 == elem.id2)
		new_task = false;
	}
	
	if(new_task)
	    tasks_to_assign.push_back(elem);
	
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
    
//     for(auto elem : vect_msg.task_vect)
//     {
// 	ROS_INFO_STREAM("The task_manager is publishing the task to assign: "<< elem.name << " whit the couple " << elem.id1 << " - " << elem.id2);
//     }

}



void ReAssFunc(task_assign::robot msg, Assign ass)
{
    // se sono lontana da taska e non ho ancora eseguito taska
    if(!msg.taska && getDistance(msg.x, msg.y, ass.task.x1, ass.task.y1) > SEC_DIST)
    {
	robots_in_execution = deleteRob(msg.name, robots_in_execution);
	robots_in_recharge.push_back(msg);
	tasks_in_execution = deleteTask(ass.task.name, tasks_in_execution);
	tasks_to_assign.push_back(ass.task);
	Catalogo_Ass = deleteAss(msg.name, Catalogo_Ass);
	
	CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
	CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
	
	publishMasterIn();
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
    
    if(msg->status)
    {	
	// vedo se il robot è già in uno dei vettori
	for(auto elem : available_robots)
	{
	    if(elem.name == msg->name)
	    {
		// il robot è già in available_robots
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
		    // il robot è già in available_robots
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
		    // il robot è già in robots_in_execution
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
		    // il robot è già in robots_in_recharge
		    in_charge = true;
		    break;
		}
	    }
	}
	
   
	if(!available && !avail_rech && !in_execution && !in_charge && msg->b_level > BATTERY_THR)
	{
	    available_robots.push_back(*msg);
	    CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
	    
	    if(tasks_to_assign.size()>0)
	    {
		publishMasterIn();
	    }
	}
	
	else if(!available && !avail_rech && !in_execution && !in_charge && msg->b_level <= BATTERY_THR)
	{
	    robots_in_recharge.push_back(*msg);
	    CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
	    
	    if(recharge_points.size()>0)
	    {
		publishMasterIn();
	    }
	}
	
	else if(available)
	{	    	    
	    if(msg->b_level <= BATTERY_THR)
	    {
		    robots_in_recharge.push_back(*msg);
		    CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
		    available_robots = deleteRob(msg->name, available_robots);
	    }
	    else
		CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);	      
	}
	
	else if(avail_rech)
	{	    	    
	   CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);      
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
			publishExecTask();
			tasks_in_execution = deleteTask(ass.task.name, tasks_in_execution);
			robots_in_execution = deleteRob(msg->name, robots_in_execution);
			Catalogo_Ass = deleteAss(msg->name, Catalogo_Ass);
			
			if(msg->b_level > BATTERY_THR)
			{
			    available_robots.push_back(*msg);
			    CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
			    if(tasks_to_assign.size()>0)
			    {
				publishMasterIn();
			    }
			}
			else
			{
			    robots_in_recharge.push_back(*msg);
			    CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
			    if(recharge_points.size()>0)
			    {
				publishMasterIn();
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
			    CalcTex(rt_info_vect, robots_in_execution, tasks_in_execution, Mappa, 1);
			    
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
			CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
			if(tasks_to_assign.size()>0)
			{
			    publishMasterIn();
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
			    CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 1);
			    
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
    }
    // se il robot è rotto
    else
    {
	// TODO modulo di salvataggio
    }
}




// Function che mette in un oggetto di tipo Assign (che verrà messo poi nel Catalogo_Ass) il robot e il task di un assignment 
// e il percorso che deve fare il robot per raggiungere il task (prima fino task_a e poi da task_a a task_b)
Assign MinPath(SmartDigraph& Map, task_assign::rt r_t)
{
    Assign ass;   
    SmartDigraph::ArcMap<double> len(Map);
    SmartDigraph::NodeMap<float> coord_x(Map);
    SmartDigraph::NodeMap<float> coord_y(Map);
    SmartDigraph::Node rob;
    SmartDigraph::Node taska;
    SmartDigraph::Node taskb;
    task_assign::waypoint tmp;
    vector<task_assign::waypoint> path;
    vector<SmartDigraph::Node> path_node;
    SmartDigraph::Arc arc;
  
  
    ass.rob = r_t.robot;
    ass.task = r_t.task;
    
    ass.path_tot.r_name = r_t.robot.name;
    ass.path_tot.t_name = r_t.task.name;
    ass.path_tot.id_a = r_t.task.id1;
    ass.path_tot.id_b = r_t.task.id2;
    
    rob = SmartDigraph::nodeFromId(r_t.robot.id);
    taska = SmartDigraph::nodeFromId(r_t.task.id1);
    taskb = SmartDigraph::nodeFromId(r_t.task.id2);
 
    Dijkstra<SmartDigraph, SmartDigraph::ArcMap<double>> dijkstra_test(Map,len);
    
    

    // prima parte del task
    // ora scrivo path_a prendendo dalla mappa la lista di waypoint che vanno dalla posizione del robot alla posizione di 
    // task_a di r_t

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
	
// 	// il tempo di esecuzione è la somma dei pesi di tutti gli archi del path trovato
// 	double dist(0.0);
// 	for(int i=0; i<path_node.size()-1; i++)
// 	{
// 	    arc = lemon::findArc(Map,path_node[i],path_node[i+1]);
// 	    dist += 1/VELOCITY*len[arc];
// 	}
	
    }
    else
    {
	tmp.y = r_t.robot.x;
	tmp.x = r_t.robot.y;
	path.push_back(tmp);
    }

    ass.path_tot.path_a = path;
    path.clear();
    path_node.clear();
       
    
    
    // seconda parte del task

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
	
// 	// il tempo di esecuzione è la somma dei pesi di tutti gli archi del path trovato
// 	double dist(0.0);
// 	for(int i=0; i<path_node.size()-1; i++)
// 	{
// 	    arc = lemon::findArc(Map,path_node[i],path_node[i+1]);
// 	    dist += 1/VELOCITY*len[arc];
// 	}
	
    }
    // altrimenti taska coincide con taskb, questo significa che il task è un punto di ricarica
    else
    {
	tmp.y = r_t.task.x1;
	tmp.x = r_t.task.y1;
	path.push_back(tmp);
    }
    
    ass.path_tot.path_b = path;
    path.clear();
    path_node.clear();
	    
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
		
		
// 		ass = MapToCatal(GlobMap, rt, 1);
		ass = MinPath(Mappa, rt);
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
		
		robots_in_exec_rech.push_back(rt.robot);
		robots_in_recharge = deleteRob(rt.robot.name, robots_in_recharge);
		
		recharge_points_busy.push_back(rt.task);
		recharge_points = deleteTask(rt.task.name, tasks_to_assign);
		
// 		ass = MapToCatal(GlobMap, rt, 0);
		ass = MinPath(Mappa, rt);
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
}



// Pubblica ai robot che devono andare in carica il loro punto di ricarica
void publishRecharge()
{
    task_assign::assignment msg;
    
    msg.assign_vect = robRech_vect;
    
    sleep(1);
    recharge_pub.publish(msg);
}



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
//     rob_ass_pub = node.advertise<task_assign::vect_robot>("rob_assign_topic", 10);
//     rob_rech_pub = node.advertise<task_assign::vect_robot>("rob_recharge_topic", 10);
//     task_ass_pub = node.advertise<task_assign::vect_task>("task_assign_topic", 10);
//     rob_info_pub = node.advertise<task_assign::vect_info>("rob_info_topic", 10);
//     rech_info_pub = node.advertise<task_assign::vect_info>("rech_info_topic", 10);
    assignment_pub = node.advertise<task_assign::assignment>("assignment_topic", 10);
    recharge_pub = node.advertise<task_assign::assignment>("recharge_topic", 10);
    
    marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    reass_pub = node.advertise<task_assign::glpk_in>("glpk_in_topic", 10);
    
    
    sleep(1);

    
    
    
    // Carica il grafo dal file .lgf, e lo mette nel grafo orientato Mappa (var globale)
    SmartDigraph::NodeMap<float> coord_x(Mappa);
    SmartDigraph::NodeMap<float> coord_y(Mappa);
    SmartDigraph::NodeMap<int> id(Mappa);
    SmartDigraph::NodeMap<dim2::Point<float> > coords(Mappa);
    SmartDigraph::ArcMap<double> len(Mappa);

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
	while(!new_assign && !new_in_rech && ros::ok())
	{
	    ros::spinOnce();
	    rate.sleep();	  
	}
      
	if(new_assign && ros::ok())
	{
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



// Questo blocco va modificato considerando la mappa in lemon e l'alg. di Dijkstra per calcolare i tex sul grafo
//-----------------------------------------------------------------------------------------------------------------//

// double CalcPath(vector<pair<double,double>> wpoints)
// {
//     double dist(0);
//     
//     for(int i=0; i<wpoints.size()-1; i++)
//     {
// 	dist += getDistance(wpoints[i].first,wpoints[i].second,wpoints[i+1].first,wpoints[i+1].second);
//     }
//     
//     return dist;
// }
// 
// 
// 
// // Function che calcola il path partendo dalla posizione di un robot che non sta tra i wp della mappa
// // idea: cerca il wp più vicino sulla mappa e somma la distanza fino al wp con il resto del path (dal wp al task)
// vector<pair<double,double>> CalcDistMap(double rx, double ry, double tx, double ty, vector<mappa> maps)
// {
//     double min_dist(1000);
//     double dist(0.0);
//     pair<double,double> coord;
//     coord = make_pair(rx, ry);
//     mappa min_elem;
// 
//     
//     for(auto elem : maps)
//     {
//     	dist = getDistance(rx, ry, elem.start.first, elem.start.second);
// 	if(dist <= SEC_DIST && tx == elem.end.first && ty == elem.end.second)
// 	{
// 	    if(dist < min_dist)
// 	    {
// 	    	min_dist = dist;
// 	    	min_elem.start = elem.start;
// 	    	min_elem.end = elem.end;
// 	    	min_elem.wpoints = elem.wpoints;
// 	    }
// 	}
//     }
//     
//     min_elem.wpoints.insert(min_elem.wpoints.begin(), coord);
//     min_elem.start = coord;
//     GlobMap.push_back(min_elem);
//     
//     return min_elem.wpoints;
// }
// 
// 
// 
// // Function che calcola il tempo necessario a ciascun robot per raggiungere tutti i task
// // se i=0 calcolo i percorsi nell'istante "iniziale", se i=1 calcolo t_ex negli altri istanti
// vector<task_assign::info> CalcTex(vector<task_assign::robot> robots, vector<task_assign::task> tasks, vector<mappa> maps, int i)
// {
//     vector<task_assign::info> tex;
//     task_assign::info info;
//     double time_a(0);
//     double time_b(0);
//     bool in_map(false);
//     
//     // idea 1
//     for(auto rob : robots)
//     {
// 	// vedo se elem sta già in task_to_assign
// 	for(auto task : tasks)
// 	{
// 	    info.r_name = rob.name;
// 	    info.t_name = task.name;
// 	    
// 	    for(auto elem : maps)
// 	    {
// 		if(rob.x == elem.start.first && rob.y == elem.start.second && task.x1 == elem.end.first && task.y1 == elem.end.second)
// 		{
// 		    in_map = true;
// 		    time_a = 1/VELOCITY*CalcPath(elem.wpoints);
// 		    break;
// 		}
// 	    }
// 	    if(!in_map)
// 	    {
// 		time_a = 1/VELOCITY*CalcPath(CalcDistMap(rob.x, rob.y, task.x1, task.y1, GlobMap));
// 	    }
// 	    in_map = false;
// 	    
// 	    for(auto elem : maps)
// 	    {
// 		if(task.x1 == elem.start.first && task.y1 == elem.start.second && task.x2 == elem.end.first && task.y2 == elem.end.second)
// 		{
// 		    time_b = 1/VELOCITY*CalcPath(elem.wpoints);
// 		    break;
// 		}
// 	    }
// 	    
// 	    if(!i)
// 	    {
// 		info.t_ex0 = time_a + task.wait1 + time_b + task.wait1;
// 		info.t_ex = time_a + task.wait1 + time_b + task.wait1;
// 	    }
// 	    else
// 		info.t_ex = time_a + task.wait1 + time_b + task.wait1;
// 	    
// 	    tex.push_back(info);
// 	}
//     }
//     
//     return tex;
// }


//-----------------------------------------------------------------------------------------------------------------//


// Questo blocco va modificato considerando la mappa in lemon e l'alg. di Dijkstra per calcolare i path sul grafo
//-----------------------------------------------------------------------------------------------------------------//

// // Function che copia una lista di w.p. dalla mappa globale in una lista di oggetti task_assign/waypoint
// vector<task_assign::waypoint> CopiaPath(vector<pair<double,double>> wpoints)
// {
//     task_assign::waypoint wp;
//     vector<task_assign::waypoint> path;
//     
//     for(auto pair : wpoints)
//     {
// 	wp.x = pair.first;
// 	wp.y = pair.second;
// 	
// 	path.push_back(wp);
//     }
//     
//     
//     return path;
// }
// 
// 
// 
// // Function che mette in un oggetto di tipo Assign (che verrà messo poi nel Catalogo_Ass) il robot e il task di un assignment 
// // e il percorso che deve fare il robot per raggiungere il task (prima fino task_a e poi da task_a a task_b)
// Assign MapToCatal(vector<mappa> Map, task_assign::rt r_t, int i)
// {
//     vector<task_assign::waypoint> path;
//     task_assign::waypoint wp;
//     Assign ass;
//     bool in_map(false);
//   
//   
//     ass.rob = r_t.robot;
//     ass.task = r_t.task;
//     
//     ass.path_tot.r_name = r_t.robot.name;
//     ass.path_tot.t_name = r_t.task.name;
//     ass.path_tot.id_a = r_t.task.id1;
//     ass.path_tot.id_b = r_t.task.id2;
//     
//     //ora scrivo path_a prendendo dalla mappa la lista di waypoint che vanno dalla posizione del robot alla posizione di task_a di r_t
//     for(auto elem : Map)
//     {
// 	if(r_t.robot.x == elem.start.first && r_t.robot.y == elem.start.second && r_t.task.x1 == elem.end.first && r_t.task.y1 == elem.end.second)
// 	{
// 	    in_map = true;
// 	    ass.path_tot.path_a = CopiaPath(elem.wpoints);
// 	    break;
// 	}
//     }
//     if(!in_map)
//     {
// 	ass.path_tot.path_a = CopiaPath(CalcDistMap(r_t.robot.x, r_t.robot.y, r_t.task.x1, r_t.task.y1, GlobMap));
//     }
// 
//     //se i=1 sto usando la function per gli assignment, altrimenti per i recharge e quindi non serbe task b
//     if(i)
//     {
// 	//ora scrivo path_b prendendo dalla mappa la lista di waypoint che vanno dalla posizione di task_a all posizione di task_b di r_t
// 	for(auto elem : Map)
// 	{
// 	    if(r_t.task.x1 == elem.start.first && r_t.task.y1 == elem.start.second && r_t.task.x2 == elem.end.first && r_t.task.y2 == elem.end.second)
// 	    {
// 		ass.path_tot.path_b = CopiaPath(elem.wpoints);	    
// 		break;
// 	    }
// 	}
//     }
//     
//     
//     return ass;
// }

//-----------------------------------------------------------------------------------------------------------------//


// // Pubblica al master il vettore dei task da eseguire task_to_assign
// void publishTaskToAssign()
// {
//     task_assign::vect_task vect_msg; 
//     
//     vect_msg.task_vect = tasks_to_assign;
// 
//     // Wait for the publisher to connect to subscribers
//     sleep(1.0);
//     task_ass_pub.publish(vect_msg);
//     
// //     for(auto elem : vect_msg.task_vect)
// //     {
// // 	ROS_INFO_STREAM("The task_manager is publishing the task to assign: "<< elem.name << " whit the couple " << elem.id1 << " - " << elem.id2);
// //     }
// }
// 
// 
// 
// // Pubblica al master su "rob_assign_topic" il vettore dei robot da assegnare
// void publishRobotToAssign()
// {
//     task_assign::vect_robot vect_msg;
//     
//     vect_msg.robot_vect = available_robots;
//     
//     sleep(1);
//     rob_ass_pub.publish(vect_msg);
// }
// 
// 
// 
// // Pubblica al master su "rob_assign_topic" il vettore dei robot da assegnare
// void publishRobInRecharge()
// {
//     task_assign::vect_robot vect_msg;
//     
//     vect_msg.robot_vect = robots_in_recharge;
//     
//     sleep(1);
//     rob_ass_pub.publish(vect_msg);
// }
// 
// 
// 
// // Pubblica al master su "rob_info_topic" le info relative ai robot (tutti: già assegnati e da assegnare), pubblica tex0_info_vect
// void publishRobotInfo()
// {
//     task_assign::vect_info vect_msg;
//     
//     vect_msg.info_vect = rob_info_vect;
//     vect_msg.info0_vect = rob_info0_vect;
//     
//     sleep(1);
//     rob_info_pub.publish(vect_msg);
//    
// }
// 
// 
// 
// // Pubblica al master i tempi richiesti da ogni ogni robot per raggiungere ciascun punto di ricarica
// void publishInfoRecharge()
// {
//     task_assign::vect_info vect_msg;
//     
//     vect_msg.info_vect = rech_info_vect;
// //     vect_msg.info0_vect = rech_info0_vect;
//     
//     sleep(1);
//     rech_info_pub.publish(vect_msg);
// }
