// Nodo che modella l'agente master
// il nodo sta sempre in ascolto dei topics di cui è subscriber, chiamando una funzione di publish
// direttamente dalla callback in cui fa assignment



#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "task_assign/AgentStatus.h"  


using namespace std;

std::string robot_name;

ros::Subscriber robot_status_sub;
ros::Subscriber task_status_sub;
ros::Publisher assignment_pub;
ros::Subscriber assignment_sub;

// agent structure;
struct agent
{
    ros::Time t;
    string id;
    bool ready;
    bool status;
    string type;
    string assign;
    float x;
    float y;
    float theta;
};


struct agent task, robot;
struct agent rToAssign, tToAssign;


map<string,string> map_assignment;  // mappa assignment robot-task

map<string,agent> robots_buffer;
map<ros::Time,string> robots_time;
map<string,agent> tasks_buffer;
map<ros::Time,string> tasks_time;


int NUM_SPORTELLI;
bool giaInAssign = false;




inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}




// Il master pubblica su "assignment_topic" il messaggio contenente l'assignment, lo status del task viene messo in tutti i campi del 
// messaggio ad eccezione del campo robot_assign in cui viene messo lo status del robot corrispondente
void publishAssignment() 
{
    task_assign::AgentStatus status_msg; 
    
    status_msg.header.stamp = ros::Time::now();
    status_msg.t = tToAssign.t;
    status_msg.robot_id = tToAssign.id;
    status_msg.is_ready = tToAssign.ready;
    status_msg.status = tToAssign.status; 
    status_msg.type = tToAssign.type;
    status_msg.x = tToAssign.x;
    status_msg.y = tToAssign.y;
    status_msg.theta = tToAssign.theta;
    
    //pubblico robot_assign
    status_msg.robot_assign.t = rToAssign.t;
    status_msg.robot_assign.id = rToAssign.id;
    status_msg.robot_assign.ready = rToAssign.ready;
    status_msg.robot_assign.status = rToAssign.status;  
    status_msg.robot_assign.type = rToAssign.type;
    status_msg.robot_assign.x = rToAssign.x;
    status_msg.robot_assign.y = rToAssign.y;
    status_msg.robot_assign.theta = rToAssign.theta;

    // Wait for the publisher to connect to subscribers
    sleep(1.0);
    assignment_pub.publish(status_msg);
    
    ROS_INFO_STREAM("Master is publishing the assignment: "<< tToAssign.id << " - " << rToAssign.id);
}



// Callback con cui il master legge "assignment_topic" per vedere se ci sono robot che hanno finito, e quindi task che si liberano
// In corrispondenza dei task liberi il master rimette a false il campo status della struttura_agente_task corrispondente in 
// tasks_buffer e elimina la coppia che ha terminato da map_assignment
void FreeCallback(const task_assign::AgentStatus::ConstPtr& status_msg)
{
    if(status_msg->is_ready && !status_msg->status && !status_msg->robot_assign.status)
    {
	ROS_INFO_STREAM("Master is listening the ended assignment: "<< status_msg->robot_id << " - " << status_msg->robot_assign.id);
	
	tasks_buffer[status_msg->robot_id].status=false;
	
	// se rimetto anche lo status del robot a false, posso assegnare nuovi 
// 	robots_buffer[status_msg->robot_assign.id].status=false;
	
	map_assignment.erase(status_msg->robot_assign.id);
    }
}



// Callback con cui il master legge i topics "robot_arrival_topic" e "task_arrival_topic"
// Il master registra l'arrivo di task e robot in rispettivi buffer, in cui gli agenti sono ordinati in base al tempo di arrivo:
//     -robots_time: mappa tempo_di_arrivo-nome_robot
//     -robots_buffer: mappa nome_robot-struttura_agente_robot
//     -tasks_time: mappa tempo_di_arrivo-nome_task
//     -tasks_buffer: mappa nome_task-struttura_agente_task
// Dopo aver atteso l'arrivo di tutti i task disponibili, il master sceglie i candidati per l'assignment (strutture agente rToAssign e
// tToAssign). Se i candidati non sono già presenti nella mappa, e se il master ha fatto fin'ora un numero di assignment inferiore al 
// numero di task liberi, i candidati vengono messi in map_assignment (mappa nome_robot-nome_task) e il loro campo status viene messo 
// a true (il master modifica la struttura_agente_robot e la struttura_agente_task corrispondenti dentro i buffers rispettivi)
// Poi viene pubblicato su "assignment_topic" (con publishAssignment()) il messaggio in cui si ha il task con il robot associato 
// (il cui status è dentro msg.robot_assign)
void TotalCallback(const task_assign::AgentStatus::ConstPtr& status_msg)
{
    //check: deve essere arrivato qualcosa
    if(status_msg->is_ready && status_msg->x!=-1 && status_msg->y!=-1 && status_msg->theta!=200)
    {
        ROS_INFO_STREAM("The master is listening the agent " << status_msg->robot_id << " with status " << BoolToString(status_msg->status));
	
	map<string,string>::iterator it;
	for (it=map_assignment.begin(); it!=map_assignment.end(); ++it)
	{
	    if(status_msg->robot_id==it->first || status_msg->robot_id==it->second) break;
	}
	
	if(it==map_assignment.end()) 
	{

	    if(status_msg->type=="task" && !status_msg->status)
	    {
		int counter_t=0;
		for(auto elem: tasks_buffer)
		{
		    if(status_msg->robot_id==elem.first) break;
		    counter_t++;
		}
		if(counter_t==tasks_buffer.size())
		{
		    task.id = status_msg->robot_id;
		    task.ready = status_msg->is_ready;
		    task.status = status_msg->status;
		    task.type = status_msg->type;
		    task.x = status_msg->x;
		    task.y = status_msg->y;
		    task.theta = status_msg->theta;
		    
		    tasks_buffer[status_msg->robot_id] = task;
		    tasks_time[status_msg->t] = task.id;
		}
	    }
	    else if(status_msg->type=="robot" && !status_msg->status)
	    {
		int counter_r=0;
		for(auto elem: robots_buffer)
		{
		    if(status_msg->robot_id==elem.first) break;
		    counter_r++;
		}
		if(counter_r==robots_buffer.size())
		{
		    robot.id = status_msg->robot_id;
		    robot.ready = status_msg->is_ready;
		    robot.status = status_msg->status;
		    robot.type = status_msg->type;
		    robot.x = status_msg->x;
		    robot.y = status_msg->y;
		    robot.theta = status_msg->theta;
		    
		    robots_buffer[status_msg->robot_id] = robot;
		    robots_time[status_msg->t] = robot.id;
		}
	    }
	    
	    if(robots_time.size()==0) ROS_INFO_STREAM("Central node is publishing robots_buffer that's empty");
	    for(auto elem : robots_time)
	    {
		ROS_INFO_STREAM("Central node is publishing robots_buffer "<< elem.second);
	    }
	    
	    if(tasks_time.size()==0) ROS_INFO_STREAM("Central node is publishing tasks_buffer that's empty");
	    for(auto elem : tasks_time)
	    {
		ROS_INFO_STREAM("Central node is publishing tasks_buffer "<< elem.second);
	    }
	    
	    
		
	    // si dovrebbe avere NUM_SPORTELLI = tasks_buffer.size() e non fissato all'inizio 
	    if(tasks_buffer.size()>=NUM_SPORTELLI)
	    {
		if(robots_buffer.size()>0)
		{
		    //prendo il primo robot in coda con status false e lo metto in rToAssign
		    int counter_r=0;
		    struct agent current_r;
		    for(auto elem: robots_time)
		    {
			if(!(robots_buffer[elem.second]).status)
			{
			    current_r = robots_buffer[elem.second];
			    break;
			}
			counter_r++;
		    }
		    if(counter_r!=robots_time.size())
			rToAssign = current_r;
		}

	      
		if(tasks_buffer.size()>0)
		{
		    //prendo il primo task in coda con status false e lo metto in tToAssign
		    int counter_t=0;
		    struct agent current_t;
		    for(auto elem: tasks_time)
		    {
			if(!(tasks_buffer[elem.second]).status)
			{
			    current_t = tasks_buffer[elem.second];
			    break;
			}
			counter_t++;
		    }
		    if(counter_t!=tasks_time.size())
			tToAssign = current_t;
		}
	      
	      
		if(rToAssign.id!="" && tToAssign.id!="")
		{   
		    giaInAssign = false;
		    for(auto elem : map_assignment)
		    {
			if(rToAssign.id==elem.first || tToAssign.id==elem.second)
			  giaInAssign = true;
		    }
		    if(map_assignment.size()<NUM_SPORTELLI && !giaInAssign)
		    {
			rToAssign.status=true;
			tToAssign.status=true;
			
			map_assignment[rToAssign.id]=tToAssign.id;
		    
			robots_buffer[rToAssign.id].status=true;
			tasks_buffer[tToAssign.id].status=true;
		    
			publishAssignment();
		    }
		}
	      
		  
		for(auto elem : map_assignment)
		{
		    ROS_INFO_STREAM("In map_assignment c'e': "<< elem.first <<" - "<<elem.second);
		}
	      
	    }
	}
    }
}




int main(int argc, char **argv)
{
    NUM_SPORTELLI = atoi(argv[1]);
  
    // Initialize the node
    ros::init(argc, argv, "central_node");
    ros::NodeHandle node;
    
    task_status_sub = node.subscribe("task_arrival_topic", 20, &TotalCallback);   
    robot_status_sub = node.subscribe("robot_arrival_topic", 20, &TotalCallback);
    
    assignment_pub = node.advertise<task_assign::AgentStatus>("assignment_topic", 10);
    assignment_sub = node.subscribe("assignment_topic", 20, &FreeCallback);
    sleep(1);
    
    ros::Rate rate(10);
    while (ros::ok()) 
    {
	ros::spinOnce();
	rate.sleep();
    }



    return 0;
}
