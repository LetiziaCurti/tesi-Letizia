// Nodo che modella il master


#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"
#include "task_assign/vect_robot.h"
#include "task_assign/rt_vect.h"
#include "task_assign/vect_info.h"

#include <stdio.h>            /* C input/output                       */
#include <stdlib.h>           /* C standard library                   */
#include <glpk.h>             /* GNU GLPK linear/mixed integer solver */
#include <yaml-cpp/yaml.h>


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

ros::Publisher assignment_pub;
ros::Subscriber rob_ass_sub;
ros::Subscriber rob_info_sub;
ros::Subscriber rob_ini_sub;
ros::Subscriber task_ass_sub;


vector<task_assign::task> task_to_assign;    	//T: vettore dei task da assegnare che cambia nel tempo (di dim m(k))
vector<task_assign::robot> robot_to_assign;    	//R: vettore dei robot per l'assegnazione che cambia nel tempo (di dim n(k))
vector<task_assign::info> tex_info_vect;	//vettore dei tempi di esecuzione di ciascun robot rispetto a tutti i task
vector<task_assign::info> tex0_info_vect;	//vettore dei tempi di esecuzione di ciascun robot rispetto a tutti i task(al tempo 0)
vector<task_assign::robot> delete_rob;		//robot esclusi dall'assignment perché superano le soglie




// Legge su "task_assign_topic" il vettore dei task da eseguire m(k)
void TaskToAssCallback(const task_assign::vect_task::ConstPtr& msg)
{
    bool add_task(true);
    
    for(auto elem : msg->task_vect)
    {
	// vedo se elem sta già in task_to_assign
	for(auto newel : task_to_assign)
	{
	    if(newel.name1 == elem.name1 && newel.name2 == elem.name2)
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



// Legge su "rob_info_topic" le info relative ai robot (tutti: già assegnati e da assegnare)
void RobInfoCallback(const task_assign::vect_info::ConstPtr& msg)
{
    // non è importante l'ordine
    for(auto elem : msg->info_vect)
    {
	tex_info_vect.push_back(elem);
    }
}



// // Legge su "rob_ini_topic" le info relative ai robot al tempo 0 (tutti: già assegnati e da assegnare)
// void RobIniCallback(const task_assign::vect_info::ConstPtr& msg)
// {
//     // non è importante l'ordine
//     for(auto elem : msg->info_vect)
//     {
// 	tex0_info_vect.push_back(elem);
//     }
// }



// Pubblica al motion planner gli assignments task-robot
void publishAssign(vector<vector<int>> S)
{
    task_assign::rt_vect vect_msg;
    task_assign::rt msg;
    
    for(int i=0; i<robot_to_assign.size(); i++)
    {
	for(int j=0; j<task_to_assign.size(); j++)
	{
	    if(S[i][j]==1)
	    {
		msg.r_name = robot_to_assign[i].name;
		msg.task = task_to_assign[j];
		vect_msg.rt_vect.push_back(msg);
	    }
	}
    }
    
//     sleep(1);
    assignment_pub.publish(vect_msg);
}



// Function che calcola la Utility Function
vector<double> calcUFun(vector<task_assign::task> t_ass, vector<task_assign::robot> r_ass, vector<task_assign::info> ex_t, vector<task_assign::info> ex_t_0)
{
    vector<double> UF;
    double P(0);
    double C(0);
    double U(0);
    double b_lev(0);
    double ar_t(0);
    double t_ex_0(0);
    double t_ex(0);
    
    
    for(auto robot : r_ass)
    {
	b_lev = robot.b_level;
	 
	for(auto task : t_ass)
	{
	    C = 1/b_lev;
	
	    ar_t = task.ar_time;
	    P = 1/ar_t;
	  
	    for(auto ex0 : ex_t_0)
	    {
		if(task.name == ex0.t_name && robot.name == ex0.r_name)
		{
		    t_ex_0 = ex0.t_ex;
		    P += 1/t_ex_0;
		    C -= t_ex_0;
		}
		break;
	    }
	    for(auto ex : ex_t)
	    {
		if(task.name == ex.t_name && robot.name == ex.r_name)
		{
		    t_ex = ex.t_ex;
		    C += t_ex;
		}
		break;
	    }
	    
	    U = P-C;
	    if(U<=0)
		U=0;
	    
	    UF.push_back(U);
	}
    }
    
    return UF;
}



// Function che genera una matrice identità di dimensione m
vector<vector<int>> makeId(int m)
{
    vector<vector<int>> matId(m, vector<int> (m,0));
    
    for(int i=0; i<m; i++)
    {
	for(int j=0; j<m; j++)
	{
	    if(i==j)
		matId[i][j] = 1;
	    else
		matId[i][j] = 0;
	}
    }
    
    return matId;
}



// Function che crea la matrice B dei coefficienti del nostro MILP problem
vector<vector<int>> makeCoeff(int n, int m)
{
    vector<vector<int>> B(m+n+1, vector<int> (m*n,0));
    
    int index(1);
    int count(0);    
    for(int i=0; i<n; i++)
    {
	for(int j=0; j<m*n; j++)
	{
	    if(i+1==index)
		B[i][j] = 1;
	    else
		B[i][j] = 0;
	    
	    count++;

	    if(count==m)
	    {
		index++;
		count = 0;
	    }
	}
	index = 1;
    }
    
    index = 1;
    count = 0;
    vector<vector<int>> ID = makeId(m);
    for(int i=n; i<n+m; i++)
    {
	for(int j=0; j<m*n; j++)
	{
	    B[i][j] = ID[(i-n)][(j-(index-1)*m)];
	    count++;

	    if(count==m)
	    {
		index++;
		count = 0;
	    }
	}
	index = 1;
    }
    
    int i = n+m;
    for(int j=0; j<m*n; j++)
    {
	B[i][j] = 1;
    }
    
    
    return B;
}



double z;
// Function che risolve il TA problem dando in uscita la matrice degli assignments
vector<vector<int>> TASolver(int n, int m, vector<double> U)
{
    vector<vector<int>> Sol(n, vector<int> (m,0));
    
    vector<vector<int>> B(n+m+1, vector<int> (n*m,0));
    B = makeCoeff(n,m);
     
    /* declare variables */
    glp_prob *lp;
    int ia[1+1000], ja[1+1000];
    double ar[1+1000];
    
    /* create problem */
    lp = glp_create_prob();
    glp_set_obj_dir(lp, GLP_MAX);
    
    
    /* fill problem */
    int n_rows = n+m+1;
    int n_col = n*m;
    
    // carica i limiti dei vincoli (righe)
    glp_add_rows(lp, n_rows);
    // tutte le righe hanno GLP_UP = 1
    for(int i=1; i<n_rows; i++)
    {
// 	glp_set_row_bnds(lp, i, GLP_LO, 0.0, 0.0);
	glp_set_row_bnds(lp, i, GLP_UP, 0.0, 1.0);
    }
    glp_set_row_bnds(lp, n_rows, GLP_LO, 0.0, 1.0);
    glp_set_row_bnds(lp, n_rows, GLP_UP, 0.0, Min(n,m));
    
    // carica i coefficienti delle incognite (colonne)   
    // tutte le colonne hanno GLP_LO = 0
    glp_add_cols(lp, n_col);
    for(int j=1; j<=n_col; j++)
    {
	glp_set_col_bnds(lp, j, GLP_LO, 0.0, 0.0);
// 	glp_set_col_bnds(lp, j, GLP_UP, 0.0, 1.0);
	glp_set_obj_coef(lp, j, U[j-1]);
    }
    
    // carica la matrice dei coefficienti dei vincoli
    int k(1);
    for(int i=1; i<=n_rows; i++)
    {
	for(int j=1; j<=n_col; j++)
	{
	     ia[k] = i, ja[k] = j, ar[k] = B[i-1][j-1]; /* a[1,1] = B[i][j] */
	     k++;
	}
    }
    
    
    glp_load_matrix(lp, n_rows*n_col, ia, ja, ar);
    
    /* solve problem */
    glp_simplex(lp, NULL);
    
    /* recover and display results -- creo la matrice Sol */
    int col(1);
    for(int i=0; i<n; i++)
    {
	for(int j=0; j<m; j++)
	{
	    Sol[i][j] = glp_get_col_prim(lp, col);
	    col++;
	}
    }

    z = glp_get_obj_val(lp);
    /* housekeeping */
    glp_delete_prob(lp);
    glp_free_env();
    
    
    return Sol;
}



// Function che calcola la Utility Function
vector<double> calcUFunProva(int n, int m)
{
    vector<double> U;
    
    // -f33
    if(n==3 & m==3)
    {
	U.push_back(0.1715);
	U.push_back(0.6361);
	U.push_back(0.1833);
	U.push_back(0.2912);
	U.push_back(0.5919);
	U.push_back(0.0397);
	U.push_back(0.3000);
	U.push_back(0.6461);
	U.push_back(0.1242);
    }

    // -f23
    else if(n==2 & m==3)
    {
	U.push_back(0.1715);
	U.push_back(0.6361);
	U.push_back(0.1833);
	U.push_back(0.3000);
	U.push_back(0.6461);
	U.push_back(0.1242);
    }
    
    // -f32
    else if(n==3 & m==2)
    {
	U.push_back(0.1715);
	U.push_back(0.6361);
	U.push_back(0.2912);
	U.push_back(0.5919);
	U.push_back(0.2833);
	U.push_back(0.6294);
    }
    
    return U;
}



// Function che stampa la matrice che riceve in argomento
void printMatrix(vector<vector<int>> M)
{
    vector <vector <int> >::iterator iti;
    vector <int>::iterator itj;
    
    ROS_INFO_STREAM("Matrice soluzione dell'assignment: \n");
    
    for (iti=M.begin(); iti!=M.end(); iti++)
    {
	for (itj=(*iti).begin(); itj!=(*iti).end(); itj++)
	{
	    std::cout << *itj << " ";
	}
	ROS_INFO_STREAM("\n");
    }
    
    ROS_INFO_STREAM("optimal solution "<< z << "\n");
}



int main(int argc, char **argv)
{
  
    // Initialize the node
    ros::init(argc, argv, "master");
    ros::NodeHandle node;
    
    rob_ass_sub = node.subscribe("rob_assign_topic", 20, &RobToAssCallback);
    rob_info_sub = node.subscribe("rob_info_topic", 20, &RobInfoCallback);
//     rob_ini_sub = node.subscribe("rob_ini_topic", 20, &RobIniCallback);
    task_ass_sub = node.subscribe("task_assign_topic", 20, &TaskToAssCallback);
    
    assignment_pub = node.advertise<task_assign::rt_vect>("assignment_topic", 10);
    
    sleep(1);
    
    
//     // Test delle functions
//     int n,m;
//     n=3;
//     m=3;
//     vector<double> U;
//     U = calcUFunProva(n,m);   
//     vector<vector<int>> S(n+m+1, vector<int> (n*m,0));   
//     S = TASolver(n,m,U);
//     printMatrix(S);
    
    vector<double> U;
    int n(0);
    int m(0);

    ros::Rate rate(10);
    while (ros::ok()) 
    {
	if(robot_to_assign.size() != n || task_to_assign.size() != m)
	{
	    ROS_INFO_STREAM("Robot vect size " << robot_to_assign.size() << "\n");
	    ROS_INFO_STREAM("Task vect size " << task_to_assign.size() << "\n");
	    
	    n = robot_to_assign.size();
	    m = task_to_assign.size();
		
	    if(n>0 && m>0)
	    {
		vector<vector<int>> S(n+m+1, vector<int> (n*m,0)); 
		
		U = calcUFun(task_to_assign, robot_to_assign, tex_info_vect, tex0_info_vect);
		S = TASolver(n, m, U);
		printMatrix(S);
		
		publishAssign(S);
	    }
	    else if(n==0)
		ROS_INFO_STREAM("There aren't any available robots \n");
	    else if(m==0)
		ROS_INFO_STREAM("There aren't any tasks to execute \n");
		
	}
	else
	{
	    sleep(1);
	    ROS_INFO_STREAM("There aren't neither tasks to execute nor available robots  \n");
	}
	
	ros::spinOnce();
	rate.sleep();
    }
    


    return 0;
}
