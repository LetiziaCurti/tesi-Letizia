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
#include <visualization_msgs/MarkerArray.h>
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
ros::Publisher marker_array_pub;
ros::Publisher reass_pub;

#define VELOCITY 10
#define BATTERY_THR_1 10
#define BATTERY_THR_2 15
#define SEC_DIST 4

bool new_assign(false);
bool new_in_rech(false);
bool new_task(false);
bool pub_master_in(false);
bool completed(false);
double tex0, tex;

vector<task_assign::robot> available_robots;    	//R: vettore dei robot per l'assegnazione che cambia nel tempo (di dim n(k))
vector<task_assign::robot> robots_in_execution;    	//vettore dei robot che stanno eseguendo dei task
vector<task_assign::task> tasks_to_assign;    		//T: vettore dei task da assegnare che cambia nel tempo di dimensione m(k)
vector<task_assign::task> tasks_in_execution;		//vettore dei task già in esecuzione
vector<task_assign::task> completed_tasks;		//vettore dei task completati che viene inviato al task_manager

vector<task_assign::robot> robots_in_recharge;    	//vettore dei robot disponibili all'assegnazione di punti di ricarica
vector<task_assign::robot> robots_in_exec_rech;    	//vettore dei robot che stanno andando a ricaricarsi
vector<task_assign::task> recharge_points;   		// è la lista di tutti i punti di ricarica LIBERI presenti nello scenario
vector<task_assign::task> recharge_points_busy; 


vector<task_assign::task_path> assignments_vect;	//vettore delle info da inviare ai robots (nome del task e percorso per raggiungerlo)
vector<task_assign::task_path> robRech_vect;		//vettore degli assignments robot-punto di ricarica

vector<task_assign::info> rt_info_vect;
vector<task_assign::info> rech_info_vect; 

vector<task_assign::task> obstacles;


SmartDigraph Mappa; 
SmartDigraph::NodeMap<float> coord_x(Mappa);
SmartDigraph::NodeMap<float> coord_y(Mappa);
SmartDigraph::NodeMap<int> id(Mappa);
SmartDigraph::NodeMap<dim2::Point<float> > coords(Mappa);
SmartDigraph::ArcMap<double> len(Mappa);
map<int, SmartDigraph::Node> excl_task_nodes;
map<int, SmartDigraph::Node> excl_obs_nodes;


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



int searchNode(float x, float y)
{
    int id_node;
    for (SmartDigraph::NodeIt n(Mappa); n != INVALID; ++n)
    {
	if(coord_x[n]==x && coord_y[n]==y && ros::ok())
	{
	    id_node = id[n];
	    break;
	}
    }
    
    return id_node;
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



void publishMarkerArray(vector<task_assign::task> obs_vect)
{
    visualization_msgs::MarkerArray markers_vect;
    
    for(auto elem : obs_vect)
    { 
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "motion_planner_obs";
	marker.id = elem.id1;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// 	marker.type = visualization_msgs::Marker::CUBE;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = "package://task_assign/config/1-82-3_5.stl";

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = elem.x1;
	marker.pose.position.y = elem.y1;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
// 	marker.scale.x = 1.5;
// 	marker.scale.y = 1.5;
// 	marker.scale.z = 1.5;
	marker.scale.x = 0.005;
	marker.scale.y = 0.005;
	marker.scale.z = 0.005;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 1.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.7;

	marker.lifetime = ros::Duration();
	
	markers_vect.markers.push_back(marker);
    }

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
    marker_array_pub.publish(markers_vect);
}



// Legge gli ostacoli da "obstacles_topic" 
void ObsCallback(const task_assign::vect_task::ConstPtr& msg)
{
    if(msg->task_vect.size()>0)
    {
	map<int,SmartDigraph::Node>::iterator it;
	
	for(auto elem : msg->task_vect)
	{
	    // vedo se elem sta già in obstacles
	    it = excl_obs_nodes.find(elem.id1);
	    if(it == excl_obs_nodes.end())
	    {
		ROS_INFO_STREAM("The motion_planner is storing the obstacle: "<< elem.name);
		obstacles.push_back(elem);
		excl_obs_nodes[elem.id1] = SmartDigraph::nodeFromId(elem.id1);
	    }
	}
	
	delNode(excl_obs_nodes);
	publishMarkerArray(obstacles);
    } 
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



double CalcPath(vector<task_assign::waypoint> wpoints)
{
    double dist(0);
    
    for(int i=0; i<wpoints.size()-1; i++)
    {
	dist += getDistance(wpoints[i].x,wpoints[i].y,wpoints[i+1].x,wpoints[i+1].y);
    }
    
    return dist;
}



// Function che calcola il tempo necessario a ciascun robot per raggiungere tutti i task
// se i=0 calcolo i percorsi nell'istante "iniziale", se i=1 calcolo t_ex negli altri istanti
vector<task_assign::info> CalcTex(vector<task_assign::info> info_vect, vector<task_assign::robot> robots, vector<task_assign::task> tasks, SmartDigraph &maps, int op)
{
    // vettore delle info in uscita
    vector<task_assign::info> out;
    out = info_vect;
		
	
    // creo i vettori dei nodi corrispondenti alle posizioni dei robots e dei tasks
    vector<SmartDigraph::Node> rob_start_nodes;
    vector<SmartDigraph::Node> taska_goal_nodes;
    vector<SmartDigraph::Node> taskb_goal_nodes;
    
    // metti nei vettori i nodi corrispondenti
    for(auto elem : robots)
    {
	rob_start_nodes.push_back(SmartDigraph::nodeFromId(searchNode(elem.x, elem.y)));
    }
    
    for(auto elem : tasks)
    {
	taska_goal_nodes.push_back(SmartDigraph::nodeFromId(elem.id1));	
	
	if(elem.id1 != elem.id2) //il task non è un rech. point
	    taskb_goal_nodes.push_back(SmartDigraph::nodeFromId(elem.id2));
    }
  

    
    
    //con l'alg. di Dijkstra trovo il percorso minimo che c'è tra i nodi che mi servono (quelli in corrispondenza delle 
    //posizioni di robot e task). il percorso consiste in una sequenza di nodi che viene memorizzata in un vettore, che 
    //poi viene ribaltato per avere la sequenza di wp che il robot deve percorrere per arrivare al task0
    vector<task_assign::waypoint> path;
    vector<SmartDigraph::Node> path_node;
    SmartDigraph::Arc arc;
    double time_a(0);
    double time_b(0);
    double dist(0);
    
    map<int, SmartDigraph::Node>::iterator it;
    SmartDigraph::Node n;
    map<int, SmartDigraph::Node> temp;
    
    
   
    
    for(int i=0; i<robots.size(); i++)
    {
	for(int j=0; j<tasks.size(); j++)
	{
	    task_assign::info info;
	    
	    info.r_name = robots[i].name;
	    info.t_name = tasks[j].name;
	    dist = 0;
	    	    	    	    
	    it=excl_task_nodes.find(tasks[j].id1);
	    if(it != excl_task_nodes.end())
		excl_task_nodes.erase(it);
	    if(tasks[j].id1 != tasks[j].id2)
	    {
		it=excl_task_nodes.find(tasks[j].id2);
		if(it != excl_task_nodes.end())
		    excl_task_nodes.erase(it);
	    }
	    delNode(excl_task_nodes);
// 	    temp.clear();
// 	    temp[tasks[j].id1] = SmartDigraph::nodeFromId(tasks[j].id1);
// 	    if(tasks[j].id1 != tasks[j].id2)
// 		temp[tasks[j].id2] = SmartDigraph::nodeFromId(tasks[j].id2);
// 	    insertNode(temp);

	    
	    Dijkstra<SmartDigraph, SmartDigraph::ArcMap<double>> dijkstra_test(Mappa,len);
	    
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
		
// 		// il tempo di esecuzione è la somma dei pesi di tutti gli archi del path trovato		
// 		for(int k=0; k<path_node.size()-1; k++)
// 		{
// 		    arc = lemon::findArc(Mappa,path_node[k],path_node[k+1]);
// 		    dist += len[arc];
// 		}
		dist = CalcPath(path);
// 		ROS_INFO_STREAM("Distanza tra " << robots[i].name << " - " << tasks[j].name << " : " << dist);
		time_a = dist/VELOCITY;
// 		ROS_INFO_STREAM("time_a tra " << robots[i].name << " - " << tasks[j].name << " : " << time_a);
	    }
	    else
	    {
		time_a = 0.01;
	    }
	    
	    path.clear();
	    path_node.clear();
	    
	    
	    
	    // seconda parte del task, la faccio solo se il task non è un rech. point
	    
	    if(tasks[j].id1 != tasks[j].id2) //il task non è un rech. point
	    {
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
		    
// 		    // il tempo di esecuzione è la somma dei pesi di tutti gli archi del path trovato
// 		    for(int k=0; k<path_node.size()-1; k++)
// 		    {
// 			arc = lemon::findArc(Mappa,path_node[k],path_node[k+1]);
// 			dist += len[arc];
// 		    }
		    dist = CalcPath(path);	
// 		    ROS_INFO_STREAM("Distanza tra " << robots[i].name << " - " << tasks[j].name << " : " << dist);
		    time_b = dist/VELOCITY;
// 		    ROS_INFO_STREAM("time_b tra " << robots[i].name << " - " << tasks[j].name << " : " << time_b);
		}
		else
		{
		    time_b = 0.01;
		}
	    }
	    // il task è un rech. point
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
	    for(auto elem : out)
	    {
		if(elem.r_name == robots[i].name && elem.t_name == tasks[j].name)
		{
		      elem.t_ex0 = info.t_ex0;
		      elem.t_ex = info.t_ex;
		      in = true;
		      break;
		}
	    }
	    if(!in)
		out.push_back(info);
	    
	    in = false;	    
	    
	    insertNode(excl_task_nodes);
	    excl_task_nodes[tasks[j].id1] = SmartDigraph::nodeFromId(tasks[j].id1);
	    if(tasks[j].id1 != tasks[j].id2)
		excl_task_nodes[tasks[j].id2] = SmartDigraph::nodeFromId(tasks[j].id2);
// 	    delNode(temp);
	}
    }
    
    return out;
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
	    if(newel.name == elem.name)
	    {
		new_task = false;
		break;
	    }
	}
	for(auto newel : tasks_in_execution)
	{
	    if(newel.name == elem.name)
	    {
		new_task = false;
		break;
	    }
	}
	for(auto newel : completed_tasks)
	{
	    if(newel.name == elem.name)
	    {
		new_task = false;
		break;
	    }
	}
	
	if(new_task)
	{
	    tasks_to_assign.push_back(elem);
	    ROS_INFO_STREAM("The motion_planner is storing the task to assign: "<< elem.name);
// 	    // metti nel vettore excl_task_nodes i nodi corrispondenti ai nuovi task
// 	    n = SmartDigraph::nodeFromId(elem.id1);
// 	    excl_task_nodes[elem.id1] = n;
// 	    n = SmartDigraph::nodeFromId(elem.id2);
// 	    excl_task_nodes[elem.id2] = n;
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
    msg.rech_points = recharge_points;
    msg.rob_info = rt_info_vect;
    msg.rech_rob_info = rech_info_vect;
    
    
    // Wait for the publisher to connect to subscribers
    sleep(1.0);
    reass_pub.publish(msg);

    ROS_INFO_STREAM("The motion_planner is publishing to the master: \n");
    ROS_INFO_STREAM("Robot disponibili:");
    for(auto elem : available_robots)
    {
	ROS_INFO_STREAM(elem.name);
    }
    ROS_INFO_STREAM("Task da assegnare:");
    for(auto elem : tasks_to_assign)
    {
	ROS_INFO_STREAM(elem.name);
    }
    ROS_INFO_STREAM("Robot da ricaricare:");
    for(auto elem : robots_in_recharge)
    {
	ROS_INFO_STREAM(elem.name);
    }
    ROS_INFO_STREAM("Punti di ricarica liberi:");
    for(auto elem : recharge_points)
    {
	ROS_INFO_STREAM(elem.name);
    }
//     ROS_INFO_STREAM("tempi di esec. robot-task:");
//     for(auto elem : rt_info_vect)
//     {
// 	ROS_INFO_STREAM("rob: " << elem.r_name << " - task: " << elem.t_name);
// 	ROS_INFO_STREAM("tex0: " << elem.t_ex0 << "	tex: " << elem.t_ex);
//     }
//     ROS_INFO_STREAM("tempi di esec. robot-punto di ric.:");
//     for(auto elem : rech_info_vect)
//     {
// 	ROS_INFO_STREAM("rob: " << elem.r_name << " - punto di ricarica: " << elem.t_name);
// 	ROS_INFO_STREAM("tex0: " << elem.t_ex0 << "	tex: " << elem.t_ex);
//     }

}



void ReAssFunc(task_assign::robot msg, Assign ass)
{
    ROS_INFO_STREAM(msg.name << " sta facendo reassignment");
    // se sono lontana da taska e non ho ancora eseguito taska
    if(!msg.taska && getDistance(msg.x, msg.y, ass.task.x1, ass.task.y1) > SEC_DIST)
    {
	ROS_INFO_STREAM(msg.name << " non riesce ad arrivare alla prima parte di " << ass.task.name);
	
	robots_in_execution = deleteRob(msg.name, robots_in_execution);
	robots_in_recharge.push_back(msg);
	tasks_in_execution = deleteTask(ass.task.name, tasks_in_execution);
	tasks_to_assign.push_back(ass.task);
	Catalogo_Ass = deleteAss(msg.name, Catalogo_Ass);
	
	rt_info_vect = CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
	rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
	
	pub_master_in = true;
    }
    // se ho eseguito taska e ora sono lontana sia da taska sia da taskb
    else if(msg.taska && getDistance(msg.x, msg.y, ass.task.x2, ass.task.y2) > SEC_DIST)
    {
	ROS_INFO_STREAM(msg.name << " non riesce ad arrivare alla seconda parte di " << ass.task.name);
	task_assign::task safe_task;
	
	robots_in_execution = deleteRob(msg.name, robots_in_execution);
// 	robots_in_recharge.push_back(msg);
	tasks_in_execution = deleteTask(ass.task.name, tasks_in_execution);
	Catalogo_Ass = deleteAss(msg.name, Catalogo_Ass);
	
	// creo un nuovo "task di salvataggio" in cui la prima parte è arrivare alla posizione del robot in difficoltà 
	// e la seconda parte è arrivare alla seconda parte del robot in difficoltà
	safe_task = ass.task;
	safe_task.x1 = msg.x;
	safe_task.y1 = msg.y;
	safe_task.id1 = searchNode(msg.x, msg.y);
	
	tasks_to_assign.push_back(safe_task);
	
	rt_info_vect = CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
	rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
	
	pub_master_in = true;
    } 			 
}



// Legge "status_rob_topic" 
void StatusCallback(const task_assign::robot::ConstPtr& msg)
{    
    bool in_charge(false);
    bool in_execution(false);
    bool available(false);
    bool avail_rech(false);
    map<int, SmartDigraph::Node>::iterator it;
    SmartDigraph::Node n;
    
    if(msg->status)
    {	
// 	for(auto task : tasks_to_assign)
// 	{
// 	    n = SmartDigraph::nodeFromId(task.id1);
// 	    excl_task_nodes[task.id1] = n;
// 	    n = SmartDigraph::nodeFromId(task.id2);
// 	    excl_task_nodes[task.id2] = n;
// 	}
// 	delNode(excl_task_nodes);
      
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
	
	// il robot è appena arrivato e è carico
	if(!available && !avail_rech && !in_execution && !in_charge && msg->b_level > BATTERY_THR_2)
	{
	    available_robots.push_back(*msg);
	    ROS_INFO_STREAM("il robot " << msg->name << " viene messo in available_robots");
	}
	// il robot è appena arrivato e è scarico
	else if(!available && !avail_rech && !in_execution && !in_charge && msg->b_level <= BATTERY_THR_2)
	{
	    robots_in_recharge.push_back(*msg);
	    ROS_INFO_STREAM("il robot " << msg->name << " viene messo in robots_in_recharge");
	}
	// il robot è libero
	else if(available)
	{	    	    
	    if(msg->b_level <= BATTERY_THR_2)
	    {
		    robots_in_recharge.push_back(*msg);
		    ROS_INFO_STREAM("il robot " << msg->name << " viene messo in robots_in_recharge");
		    rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
		    available_robots = deleteRob(msg->name, available_robots);
	    }
	    else
		rt_info_vect = CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);	      
	}
	// il robot deve essere caricato (sta aspettando che gli assegnino un rech. point)
	else if(avail_rech)
	{	    	    
	   rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);      
	}
	// il robot è in esecuzione
	else if(in_execution)
	{
	    bool new_compl = true;
	    // cerco il task corrispondente al robot
	    for(auto ass : Catalogo_Ass)
	    {
		if(msg->name == ass.rob.name && msg->t_name == ass.task.name)
		{
		    ROS_INFO_STREAM("il robot " << msg->name << " e' in posizione "<<floor(msg->x+0.5)<<" - "<<floor(msg->y+0.5));
		    ROS_INFO_STREAM("il robot " << msg->name << " ha livello di batteria "<<msg->b_level);
		    ROS_INFO_STREAM("il robot " << msg->name << " sta eseguendo il task "<<ass.task.name);
		    
		    // vedo se il robot è arrivato al task
// 		    if(floor(msg->x+0.5) == ass.task.x2 && floor(msg->y+0.5) == ass.task.y2)
		    if(msg->taska && msg->taskb)
		    {			
			for(auto newel : completed_tasks)
			{
			    if(newel.name == ass.task.name)
			    {
				new_compl = false;
				break;
			    }
			}
			if(new_compl)
			{
			    ROS_INFO_STREAM("il task " << ass.task.name << " e' stato completato");
			    completed_tasks.push_back(ass.task);
			    completed = true;
			    
// 			    // aggiorno la mappa dei nodi esclusi
// 			    int in(0);
// 			    for(auto el : tasks_to_assign)
// 			    {
// 				if(ass.task.id1 == el.id1)
// 				{
// 				    in = 1;
// 				    break;
// 				}
// 			    }
// 			    for(auto el : tasks_in_execution)
// 			    {
// 				if(ass.task.id1 == el.id1)
// 				{
// 				    in = 1;
// 				    break;
// 				}
// 			    }
// 			    if(!in)
// 			    {
// 				it=excl_task_nodes.find(ass.task.id1);
// 				if(it != excl_task_nodes.end())
// 				    excl_task_nodes.erase(it);
// 				it=excl_task_nodes.find(ass.task.id2);
// 				if(it != excl_task_nodes.end())
// 				    excl_task_nodes.erase(it);
// 			    }
		
			    tasks_in_execution = deleteTask(ass.task.name, tasks_in_execution);
			    robots_in_execution = deleteRob(msg->name, robots_in_execution);
			    Catalogo_Ass = deleteAss(msg->name, Catalogo_Ass);
			    
			    if(msg->b_level > BATTERY_THR_2)
			    {
				available_robots.push_back(*msg);
				rt_info_vect = CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
				if(tasks_to_assign.size()>0)
				{
				    pub_master_in = true;
				}
			    }
			    else
			    {
				robots_in_recharge.push_back(*msg);
				ROS_INFO_STREAM("il robot " << msg->name << " viene messo in robots_in_recharge");
				rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
				if(recharge_points.size()>0)
				{
				    pub_master_in = true;				
				}			    
			    }
			}
			new_compl = true;
		    }
		    // se il robot sta eseguendo il task e non ha ancora finito
		    else
		    {	    
			// verifica sul livello di batteria
			if(msg->b_level <= BATTERY_THR_1)
			{
			    // se la distanza dal task è inferiore alla soglia SEC_DIST, ce lo faccio arrivare e poi 
			    // lo manderò in carica
			    // (quindi non faccio nulla)
			  
			    //altrimenti faccio reassignment
// 			    ReAssFunc(*msg,ass);
			}
			// verifica sull'errore tra tempi di esecuzione				
			else
			{				
			    rt_info_vect = CalcTex(rt_info_vect, robots_in_execution, tasks_in_execution, Mappa, 1);
			    
			    // cerca in rt_info_vect la coppia che sta in ass, e prendi tex0 e tex
			    for(auto elem : rt_info_vect)
			    {
				if(elem.r_name == ass.rob.name && elem.t_name == ass.task.name && ros::ok())
				{
				    tex0 = elem.t_ex0;
				    tex = elem.t_ex;
				    break;
				}
			    }
// 			    // verifica della condizione di fattibilità dell'assignment
// 			    if(tex-tex0 < 0 || tex-tex0 >= 1/tex0 + 1/ass.task.ar_time - 1/msg->b_level)
// 			    {
// 				ReAssFunc(*msg,ass);
// 			    }
			}
		    }
		    
		    break;		    
		}
	    }
	}
	// il robot si sta ricaricando oppure sta andando in un punto di ricarica
	else if(in_charge)
	{	    
	    bool new_compl = true;
	    // cerco il p.to di ric. corrispondente al robot
	    for(auto ass : Catalogo_Rech)
	    {
		if(msg->name == ass.rob.name)
		{
		    // vedo se il robot si è ricaricato
		    if(msg->b_level == msg->b_level0)
		    {
			for(auto newel : recharge_points)
			{
// 			    if(newel.id1 == ass.task.id1)
			    if(newel.name == ass.task.name)
			    {
				new_compl = false;
				break;
			    }
			}
			if(new_compl)
			{
			    ROS_INFO_STREAM("il robot " << ass.rob.name << " ha finito di ricaricarsi in " << ass.task.name);
			    
			    recharge_points_busy = deleteTask(ass.task.name, recharge_points_busy);
			    robots_in_exec_rech = deleteRob(msg->name, robots_in_exec_rech);
			    Catalogo_Rech = deleteAss(msg->name, Catalogo_Rech);
			    
			    available_robots.push_back(*msg);
			    recharge_points.push_back(ass.task);
			    rt_info_vect = CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
			    rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
			    if(tasks_to_assign.size()>0)
			    {
				pub_master_in = true;			  
			    }
			}
			new_compl = true;
		    }
		    // PER ORA NON consideriamo la possibilità che il robot non riesca a raggiungere i punti di ricarica
		    // (ad es. se si scarica del tutto)
// 		    // se il robot sta andando nel punto di ricarica e non ha batteria, faccio reassignment
// 		    else if(!msg->taska)
// 		    {	    
// 			// verifica sul livello di batteria
// 			if(msg->b_level <= BATTERY_THR)
// 			{
// 			    // se la distanza dal task è inferiore alla soglia SEC_DIST, ce lo faccio arrivare e poi 
// 			    // lo manderò in carica
// 			    // (quindi non faccio nulla)
// 			  
// 			    //altrimenti faccio reassignment
// 			    ReAssFunc(*msg,ass);
// 			}
// 			// verifica sull'errore tra tempi di esecuzione				
// 			else
// 			{				
// 			    rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 1);
// 			    
// 			    // cerca in rech_info_vect la coppia che sta in ass, e prendi tex0 e tex
// 			    for(auto elem : rech_info_vect)
// 			    {
// 				if(elem.r_name == ass.rob.name && elem.t_name == ass.task.name && ros::ok())
// 				{
// 				    tex0 = elem.t_ex0;
// 				    tex = elem.t_ex;
// 				    break;
// 				}
// 			    }
// 			    
// 			    if(tex-tex0 < 0 || tex-tex0 >= 1/tex0 + 1/ass.task.ar_time - 1/msg->b_level)
// 			    {
// 				ReAssFunc(*msg,ass);
// 			    }
// 			}
// 		    }
		    
		    break;		    
		}
	    }
	}
	
	bool as = false;
	if(tasks_to_assign.size()>0 && available_robots.size()>0)
	{
	    rt_info_vect = CalcTex(rt_info_vect, available_robots, tasks_to_assign, Mappa, 0);
	    as = true;
	}
	if(recharge_points.size()>0 && robots_in_recharge.size()>0)
	{
	    rech_info_vect = CalcTex(rech_info_vect, robots_in_recharge, recharge_points, Mappa, 0);
	    as = true;
	}
	if(as)
	    pub_master_in = true;
	
// 	insertNode(excl_task_nodes);
    }
    // se il robot è rotto
    else
    {
	// modulo di salvataggio
    }
}




// Function che mette in un oggetto di tipo Assign (che verrà messo poi nel Catalogo_Ass) il robot e il task di un assignment 
// e il percorso che deve fare il robot per raggiungere il task (prima fino task_a e poi da task_a a task_b)
Assign MinPath(task_assign::rt r_t)
{
    Assign ass;   
    SmartDigraph::Node rob;
    SmartDigraph::Node taska;
    SmartDigraph::Node taskb;
    task_assign::waypoint tmp;
    vector<task_assign::waypoint> path;
    vector<SmartDigraph::Node> path_node;
    
    map<int, SmartDigraph::Node>::iterator it;
    map<int, SmartDigraph::Node> temp;
    SmartDigraph::Node n;
  
  
    ass.rob = r_t.robot;
    ass.task = r_t.task;
    
    ass.path_tot.r_name = r_t.robot.name;
    ass.path_tot.t_name = r_t.task.name;
    ass.path_tot.id_a = r_t.task.id1;
    ass.path_tot.id_b = r_t.task.id2;
    ass.path_tot.stop = false;
    
    rob = SmartDigraph::nodeFromId(searchNode( r_t.robot.x, r_t.robot.y));
    taska = SmartDigraph::nodeFromId(r_t.task.id1);
    if(r_t.task.id1 != r_t.task.id2) //il task non è un rech. point
	taskb = SmartDigraph::nodeFromId(r_t.task.id2);
    

    it=excl_task_nodes.find(r_t.task.id1);
    if(it != excl_task_nodes.end())
	excl_task_nodes.erase(it);
    if(r_t.task.id1 != r_t.task.id2) 
    {
	it=excl_task_nodes.find(r_t.task.id2);
	if(it != excl_task_nodes.end())
	    excl_task_nodes.erase(it);
    }
    delNode(excl_task_nodes);
//     temp.clear();
//     temp[r_t.task.id1] = SmartDigraph::nodeFromId(r_t.task.id1);
//     if(r_t.task.id1 != r_t.task.id2)
// 	temp[r_t.task.id2] = SmartDigraph::nodeFromId(r_t.task.id2);
//     insertNode(temp);
    
    Dijkstra<SmartDigraph, SmartDigraph::ArcMap<double>> dijkstra_test(Mappa,len);
    
    
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
// 	tmp.x = r_t.robot.x;
// 	tmp.y = r_t.robot.y;
// 	path.push_back(tmp);
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
    
    if(r_t.task.id1 != r_t.task.id2) 
    {
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
// 	    tmp.x = r_t.task.x1;
// 	    tmp.y = r_t.task.y1;
// 	    path.push_back(tmp);
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
    }
    
    
    insertNode(excl_task_nodes);
    excl_task_nodes[r_t.task.id1] = SmartDigraph::nodeFromId(r_t.task.id1);
    if(r_t.task.id1 != r_t.task.id2) 
	excl_task_nodes[r_t.task.id2] = SmartDigraph::nodeFromId(r_t.task.id2);
//     delNode(temp);
	    
    return ass;
}



// Legge "assignment_topic" 
void RTCallback(const task_assign::rt_vect::ConstPtr& msg)
{
    bool add(true);
    struct Assign ass;
    
    assignments_vect.clear();
    
    if(msg->rt_vect.size() > 0)
    {	
// 	delNode(excl_task_nodes);
	//metto le nuove coppie r-t nel catalogo, gli associo il percorso selezionandolo dalla mappa globale, metto
	// il robot in robots_in_execution e il task in tasks_in_execution
	for(auto rt : msg->rt_vect)
	{
	    ROS_INFO_STREAM("il motion planner ha ricevuto dal master la coppia r-t "<< rt.robot.name << " - " << rt.task.name);
	    ROS_INFO_STREAM("il robot ha coordinate: "<< rt.robot.x << " - " << rt.robot.y << " con id: " <<  searchNode(rt.robot.x, rt.robot.y));
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
				
		ass = MinPath(rt);
		assignments_vect.push_back(ass.path_tot);

		
		// Crea il Catalogo_Ass
		Catalogo_Ass.push_back(ass);
	    }
	    
	    add = true;
	}
	
// 	insertNode(excl_task_nodes);
    }
}



// Legge "recharge_topic" 
void RechCallback(const task_assign::rt_vect::ConstPtr& msg)
{
    bool add(true);
    struct Assign ass;
    
    robRech_vect.clear();
    
    if(msg->rt_vect.size() > 0)
    {	
// 	delNode(excl_task_nodes);
	//metto le nuove coppie r-t nel catalogo, gli associo il percorso selezionandolo dalla mappa globale, metto
	// il robot in robots_in_execution e il task in tasks_in_execution
	for(auto rt : msg->rt_vect)
	{
	    ROS_INFO_STREAM("il motion planner ha ricevuto dal master la coppia r-rech.point "<< rt.robot.name << " - " << rt.task.name);
	    ROS_INFO_STREAM("il robot ha coordinate: "<< rt.robot.x << " - " << rt.robot.y << " con id: " <<  searchNode(rt.robot.x, rt.robot.y));
	    ROS_INFO_STREAM("il rech.point ha coordinate: "<< rt.task.x1 << " - " << rt.task.y1);

	    
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
		recharge_points = deleteTask(rt.task.name, recharge_points);
		
		ass = MinPath(rt);
		robRech_vect.push_back(ass.path_tot);
		
		// Crea il Catalogo_Ass
		Catalogo_Rech.push_back(ass);
	    }
	    
	    add = true;
	}
	
// 	insertNode(excl_task_nodes);
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
	if(elem.stop)
	    ROS_INFO_STREAM("\n I'M PUBLISHING REASSIGN \n");
    }
}



// Pubblica ai robot che devono andare in carica il loro punto di ricarica
void publishRecharge()
{
    task_assign::assignment msg;
    
    msg.assign_vect = robRech_vect;
    
    sleep(1);
    recharge_pub.publish(msg);
    
    for(auto elem : msg.assign_vect)
    {
	ROS_INFO_STREAM("The motion_planner is publishing to the robot: " << elem.r_name << " the assigned rech. point " << elem.t_name);
	if(elem.stop)
	    ROS_INFO_STREAM("\n I'M PUBLISHING REASSIGN \n");     
    }
}



// i punti di ricarica sono cubi verdi
void publishMarker(task_assign::waypoint p, int id_marker)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "motion_planner_ric";
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
    task_assign::waypoint wp;
    
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
    
    for(auto elem : recharge_points)
    {
	wp.x = elem.x1;
	wp.y = elem.y1;
	wp.theta = elem.theta1;
	publishMarker(wp, elem.id1);
    }
}



void RicalcAss(Assign elem, int op)
{
    task_assign::rt rt;
    Assign temp;
    rt.robot = elem.rob;
    rt.task = elem.task;
    temp = MinPath(rt);
    
    if(op)
    {
	if(elem.path_tot.path_a.size()==temp.path_tot.path_a.size() && elem.path_tot.path_b.size()==temp.path_tot.path_b.size())
	{
	    int count(0);
	    for(int i=0; i<elem.path_tot.path_a.size(); i++)
	    {
		if(elem.path_tot.path_a[i].x==temp.path_tot.path_a[i].x && elem.path_tot.path_a[i].y==temp.path_tot.path_a[i].y)
		    count++;
		else
		    break;			      
	    }
	    if(count==elem.path_tot.path_a.size())
	    {
		int count(0);
		for(int i=0; i<elem.path_tot.path_b.size(); i++)
		{
		    if(elem.path_tot.path_b[i].x==temp.path_tot.path_b[i].x && elem.path_tot.path_b[i].y==temp.path_tot.path_b[i].y)
			count++;
		    else
			break;			      
		}
		if(count==elem.path_tot.path_b.size())
		    assignments_vect.push_back(elem.path_tot);
		else
		{
		    temp.path_tot.stop = true;
		    assignments_vect.push_back(temp.path_tot);
		    ROS_INFO_STREAM("\n I'M PUBLISHING REASSIGN \n");
		}
	    }
	    else
	    {
		temp.path_tot.stop = true;
		assignments_vect.push_back(temp.path_tot);
		ROS_INFO_STREAM("\n I'M PUBLISHING REASSIGN \n");
	    }
	}
	else
	{
	    temp.path_tot.stop = true;
	    assignments_vect.push_back(temp.path_tot);
	    ROS_INFO_STREAM("\n I'M PUBLISHING REASSIGN \n");
	}
    }
    else
    {
	task_assign::rt rt;
	Assign temp;
	rt.robot = elem.rob;
	rt.task = elem.task;
	temp = MinPath(rt);
	if(elem.path_tot.path_a.size()==temp.path_tot.path_a.size())
	{
	    int count(0);
	    for(int i=0; i<elem.path_tot.path_a.size(); i++)
	    {
		if(elem.path_tot.path_a[i].x==temp.path_tot.path_a[i].x && elem.path_tot.path_a[i].y==temp.path_tot.path_a[i].y)
		    count++;
		else
		    break;			      
	    }
	    if(count==elem.path_tot.path_a.size())
		robRech_vect.push_back(elem.path_tot);
	    else
	    {
		temp.path_tot.stop = true;
		robRech_vect.push_back(temp.path_tot);
		ROS_INFO_STREAM("\n I'M PUBLISHING REASSIGN \n");
	    }
	}
	else
	{
	    temp.path_tot.stop = true;
	    robRech_vect.push_back(temp.path_tot);
	    ROS_INFO_STREAM("\n I'M PUBLISHING REASSIGN \n");
	}
    }
}


    
    




int main(int argc, char **argv)
{ 
    // Initialize the node
    ros::init(argc, argv, "planner");
    ros::NodeHandle node;
    
    obs_sub = node.subscribe("obstacles_topic", 20, &ObsCallback);
    rt_sub = node.subscribe("rt_topic", 20, &RTCallback);
    rech_sub = node.subscribe("rech_topic", 20, &RechCallback);
    status_rob_sub = node.subscribe("status_rob_topic", 50, &StatusCallback);
    new_task_sub = node.subscribe("new_task_topic", 20, &TaskToAssCallback);
    
    exec_task_pub = node.advertise<task_assign::vect_task>("task_exec_topic", 10);   
    assignment_pub = node.advertise<task_assign::assignment>("assignment_topic", 10);
    recharge_pub = node.advertise<task_assign::assignment>("recharge_topic", 10);
    
    marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    marker_array_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    reass_pub = node.advertise<task_assign::glpk_in>("glpk_in_topic", 10);
    
    
    sleep(1);

    
    // Carico i recharge points in recharge_points e poi metto i nodi del grafo corrispondenti in excl_task_nodes
    RechPoints();   
    
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
    

    for(auto elem : recharge_points)
    {
	excl_task_nodes[elem.id1] = SmartDigraph::nodeFromId(elem.id1);
    }
    
    

    int n = obstacles.size();
    ROS_INFO_STREAM("\nCI SONO " << n << " OSTACOLI \n");
    
    ros::Rate rate(50);
    while (ros::ok()) 
    {
	ros::spinOnce();

	while(!pub_master_in && !new_assign && !new_in_rech && !completed && ros::ok())
	{
	    if(completed && ros::ok())
	    {
		sleep(1);
		publishExecTask();
		completed = false;
	    }
	    
// 	    if(obstacles.size() != n)
// 	    {
// 		delNode(excl_obs_nodes);
// 		n = obstacles.size();
// 		ROS_INFO_STREAM("\nCI SONO " << n << " OSTACOLI \n");		
// 		
// 		if(Catalogo_Ass.size()>0)
// 		{
// 		    assignments_vect.clear();
// 		    for(auto elem : Catalogo_Ass)
// 		    {
// 			RicalcAss(elem,1);
// 		    }	
// 		    publishAssign();
// 		}
// 		if(Catalogo_Rech.size()>0)
// 		{
// 		    robRech_vect.clear();
// 		    for(auto elem : Catalogo_Rech)
// 		    {
// 			RicalcAss(elem,0);
// 		    }
// 		    publishRecharge();
// 		}
// 	    }
	    
	    ros::spinOnce();
	    rate.sleep();	  
	}
	if(pub_master_in && ros::ok())
	{
	    sleep(1);
	    publishMasterIn();
	    pub_master_in = false;
	    
// 	    if(obstacles.size() != n)
// 	    {
// 		delNode(excl_obs_nodes);
// 		n = obstacles.size();
// 		ROS_INFO_STREAM("\nCI SONO " << n << " OSTACOLI \n");		
// 		
// 		if(Catalogo_Ass.size()>0)
// 		{
// 		    assignments_vect.clear();
// 		    for(auto elem : Catalogo_Ass)
// 		    {
// 			RicalcAss(elem,1);
// 		    }	
// 		    publishAssign();
// 		}
// 		if(Catalogo_Rech.size()>0)
// 		{
// 		    robRech_vect.clear();
// 		    for(auto elem : Catalogo_Rech)
// 		    {
// 			RicalcAss(elem,0);
// 		    }
// 		    publishRecharge();
// 		}
// 	    }
	}
	if(new_assign && ros::ok())
	{
// 	    if(obstacles.size() != n)
// 	    {
// 		delNode(excl_obs_nodes);
// 		n = obstacles.size();
// 		ROS_INFO_STREAM("\nCI SONO " << n << " OSTACOLI \n");		
// 		
// 		if(Catalogo_Ass.size()>0)
// 		{
// 		    assignments_vect.clear();
// 		    for(auto elem : Catalogo_Ass)
// 		    {
// 			RicalcAss(elem,1);
// 		    }	
// 		    publishAssign();
// 		}
// 	    }
// 	    else
// 	    {
		sleep(1);
		publishAssign();
// 	    }
	    
	    new_assign = false;
	}
	if(new_in_rech && ros::ok())
	{
// 	    if(obstacles.size() != n)
// 	    {
// 		delNode(excl_obs_nodes);
// 		n = obstacles.size();
// 		ROS_INFO_STREAM("\nCI SONO " << n << " OSTACOLI \n");
// 
// 		if(Catalogo_Rech.size()>0)
// 		{
// 		    robRech_vect.clear();
// 		    for(auto elem : Catalogo_Rech)
// 		    {
// 			RicalcAss(elem,0);
// 		    }
// // 		    sleep(1);
// 		    publishRecharge();
// 		}		
// 	    }
// 	    else
// 	    {
		sleep(1);
		publishRecharge();
// 	    }
	    
	    new_in_rech = false;
	}	
	if(completed && ros::ok())
	{
	    sleep(1);
	    publishExecTask();
	    completed = false;
	    
// 	    if(obstacles.size() != n)
// 	    {
// 		delNode(excl_obs_nodes);
// 		n = obstacles.size();
// 		ROS_INFO_STREAM("\nCI SONO " << n << " OSTACOLI \n");		
// 		
// 		if(Catalogo_Ass.size()>0)
// 		{
// 		    assignments_vect.clear();
// 		    for(auto elem : Catalogo_Ass)
// 		    {
// 			RicalcAss(elem,1);
// 		    }	
// 		    publishAssign();
// 		}
// 		if(Catalogo_Rech.size()>0)
// 		{
// 		    robRech_vect.clear();
// 		    for(auto elem : Catalogo_Rech)
// 		    {
// 			RicalcAss(elem,0);
// 		    }
// 		    publishRecharge();
// 		}
// 	    }
	}

	ros::spinOnce();
	rate.sleep();
    }
    
    

    return 0;
}


