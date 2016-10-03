// Nodo che modella il master


#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>            /* C input/output                       */
#include <stdlib.h>           /* C standard library                   */
#include <glpk.h>             /* GNU GLPK linear/mixed integer solver */
#include <yaml-cpp/yaml.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "task_assign/vect_task.h"
#include "task_assign/vect_robot.h"
#include "task_assign/rt_vect.h"
#include "task_assign/vect_info.h"
#include "task_assign/glpk_in.h"
#include "task_assign/glpk_sol.h"




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



ros::Subscriber reass_sub;
ros::Publisher reass_pub;

vector<task_assign::task> task_to_assign;    	//T: vettore dei task da assegnare che cambia nel tempo (di dim m(k))
vector<task_assign::robot> robot_to_assign;    	//R: vettore dei robot per l'assegnazione che cambia nel tempo (di dim n(k))
vector<task_assign::info> tex_info_vect;	//vettore dei tempi di esecuzione di ciascun robot rispetto a tutti i task
vector<task_assign::info> tex0_info_vect;	//vettore dei tempi di esecuzione di ciascun robot rispetto a tutti i task
vector<task_assign::robot> robots_in_recharge;  //R: vettore dei robot che devono essere caricati
vector<task_assign::info> rech_info_vect;
vector<task_assign::info> rech_info0_vect;

vector<task_assign::task> recharge_points;   		// è la lista di tutti i punti di ricarica presenti nello scenario, va passata dall'esterno

vector<task_assign::rt> r_t_ass;
vector<task_assign::rt> r_rech_ass;

bool do_reassign(false);



// Legge su "glpk_in_topic" i vettori delle info che vanno in ingresso all'alg. di assignment MA solo se reassign è a true
void InCallback(const task_assign::glpk_in::ConstPtr& msg)
{
    if(msg->reassign)
    {
	task_to_assign = msg->task_to_ass;
	robot_to_assign = msg->rob_to_ass;
	robots_in_recharge = msg->rob_in_rech;
	tex_info_vect = msg->rob_info;
	tex0_info_vect = msg->rob_info0;
	rech_info_vect = msg->rech_rob_info;
	rech_info0_vect = msg->rech_rob_info0;
	
	do_reassign = true;
    }
}



// pubblica l'assignment r-t
void StoVectRT(vector<vector<int>> S)
{
    task_assign::rt msg;
    
    for(int i=0; i<robot_to_assign.size(); i++)
    {
	for(int j=0; j<task_to_assign.size(); j++)
	{
	    if(S[i][j]==1)
	    {
		msg.robot = robot_to_assign[i];
		msg.task = task_to_assign[j];
		r_t_ass.push_back(msg);
	    }
	}
    }
}



// pubblica l'assignment r-p.ti di ric.
void StoVectRR(vector<vector<int>> S)
{
    task_assign::rt msg;

    for(int i=0; i<robots_in_recharge.size(); i++)
    {
	for(int j=0; j<recharge_points.size(); j++)
	{
	    if(S[i][j]==1)
	    {
		msg.robot = robots_in_recharge[i];
		msg.task = recharge_points[j];
		r_rech_ass.push_back(msg);
	    }
	}
    }
}



// Pubblica al motion planner gli assignments task-robot o robot-rech. point
void publishSol()
{
    task_assign::glpk_sol vect_msg;
    
    vect_msg.r_t_ass = r_t_ass;
    vect_msg.r_rech_ass = r_rech_ass;   
    
    sleep(1);
    reass_pub.publish(vect_msg);
}



// Function che calcola la Utility Function
vector<double> calcUFun(vector<task_assign::task> t_ass, vector<task_assign::robot> r_ass, vector<task_assign::info> ex_t, vector<task_assign::info> ex_t_0, string s)
{
    vector<double> UF;
    double P(0);
    double C(0);
    double U(0);
    double b_lev(0);
    double ar_t(0);
    double t_ex_0(0);
    double t_ex(0);
    int position(1);
    
    
    for(auto robot : r_ass)
    {
	b_lev = robot.b_level;
	 
	for(auto task : t_ass)
	{
	    if(s=="assignment")
	    {
		C = 1/b_lev;
		ar_t = task.ar_time;
		P = 1/ar_t;
	    }
	    else if(s=="recharge")
	    {
		C = b_lev;
		P = 1/position;
	    }
	

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
	
	position++;
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
    
    reass_pub = node.advertise<task_assign::glpk_sol>("glpk_sol_topic", 10);
    reass_sub = node.subscribe("glpk_in_topic", 20, &InCallback);
    
    sleep(1);
    
    
    
    vector<double> U;
    int n(0);
    int m(0);
    int r(0);
    int s(0);

    ros::Rate rate(10);
    while (ros::ok()) 
    {
	while(!do_reassign && ros::ok())
	{  
	    ros::spinOnce();
	    rate.sleep();
	}
	while(do_reassign && ros::ok())
	{  
	    // se il master deve fare assignment robot-task
	    if(robot_to_assign.size()>0)
	    {
		n = robot_to_assign.size();
		m = task_to_assign.size();
		
		vector<vector<int>> S(n+m+1, vector<int> (n*m,0)); 
		
		U = calcUFun(task_to_assign, robot_to_assign, tex_info_vect, tex0_info_vect, "assignment");
		S = TASolver(n, m, U);
		printMatrix(S);
		
		StoVectRT(S);	
		ROS_INFO_STREAM("Il master ha calcolato l'assignment di " << robot_to_assign.size() << " robot per " 
		<< task_to_assign.size() << " task" << "\n");
	    }
	    
	    if(robots_in_recharge.size()>0)
	    {
		r = robots_in_recharge.size();
		s = recharge_points.size();

		vector<vector<int>> S(r+s+1, vector<int> (r*s,0)); 
		
		U = calcUFun(recharge_points, robots_in_recharge, rech_info_vect, rech_info0_vect, "recharge");
		S = TASolver(r, s, U);
		printMatrix(S);
		
		StoVectRR(S);
		ROS_INFO_STREAM("Il master ha calcolato l'assignment di " << robot_to_assign.size() << 
		" robot per i punti di ircarica" << "\n");
	    }
	    
	    publishSol();
	    do_reassign = false;
	}
	
// 	robot_to_assign.clear();
// 	task_to_assign.clear();
// 	robots_in_recharge.clear();
	
	ros::spinOnce();
	rate.sleep();	
    }

// 	if(robot_to_assign.size() != n || task_to_assign.size() != m)
// 	{
// 	    ROS_INFO_STREAM("Robot vect size " << robot_to_assign.size() << "\n");
// 	    ROS_INFO_STREAM("Task vect size " << task_to_assign.size() << "\n");
// 	    
// 	    n = robot_to_assign.size();
// 	    m = task_to_assign.size();
// 		
// 	    if(n>0 && m>0)
// 	    {
// 		vector<vector<int>> S(n+m+1, vector<int> (n*m,0)); 
// 		
// 		U = calcUFun(task_to_assign, robot_to_assign, tex_info_vect, tex0_info_vect, "assignment");
// 		S = TASolver(n, m, U);
// 		printMatrix(S);
// 		
// 		publishSol(S,1);
// 	    }
// 	    else if(n==0)
// 		ROS_INFO_STREAM("There aren't any available robots \n");
// 	    else if(m==0)
// 		ROS_INFO_STREAM("There aren't any tasks to execute \n");
// 		
// 	}
// 	else
// 	{
// 	    sleep(1);
// 	    ROS_INFO_STREAM("There aren't neither tasks to execute nor available robots  \n");
// 	}
// 	
// 	
// 	if(robots_in_recharge.size() != r || recharge_points.size() != s)
// 	{
// 	    ROS_INFO_STREAM("Robot vect size " << robot_to_assign.size() << "\n");
// 	    ROS_INFO_STREAM("Task vect size " << task_to_assign.size() << "\n");
// 	    
// 	    r = robots_in_recharge.size();
// 	    s = recharge_points.size();
// 		
// 	    if(r>0 && s>0)
// 	    {
// 		vector<vector<int>> S(r+s+1, vector<int> (r*s,0)); 
// 		
// 		U = calcUFun(recharge_points, robots_in_recharge, rech_info_vect, rech_info0_vect, "recharge");
// 		S = TASolver(r, s, U);
// 		printMatrix(S);
// 		
// 		publishSol(S,0);
// 	    }
// 	    else if(r==0)
// 		ROS_INFO_STREAM("There aren't any robots in recharge \n");
// 	    else if(s==0)
// 		ROS_INFO_STREAM("There aren't any recharge points \n");
// 		
// 	}
// 	else
// 	{
// 	    sleep(1);
// 	    ROS_INFO_STREAM("There aren't robots in recharge  \n");
// 	}

    


    return 0;
}
