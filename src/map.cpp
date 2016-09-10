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

using namespace std;


//simulo una mappa
struct mappa
{
    pair<double,double> start;
    pair<double,double> end;
    vector<pair<double,double>> wpoints;
};

vector<mappa> GlobMap;  


mappa constructMap(pair<double,double> start, pair<double,double> end, vector<pair<double,double>> wpoints)
{
    mappa elem;
    elem.start = start;
    elem.end = end;
    elem.wpoints = wpoints;
    
    return elem;
}




int main(int argc, char **argv)
{ 
    vector<pair<double,double>> wp;
    mappa elem;
    
    // t11
    // t11 - t12
    wp.push_back(make_pair(7,8.5));
    wp.push_back(make_pair(8,8.5));
    wp.push_back(make_pair(8,10.5));
    wp.push_back(make_pair(8,12.5));
    wp.push_back(make_pair(8,15));
    wp.push_back(make_pair(10,15));
    wp.push_back(make_pair(12,15));
    wp.push_back(make_pair(14,15));
    wp.push_back(make_pair(16,15));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t11 - t21
    wp.push_back(make_pair(7,8.5));
    wp.push_back(make_pair(6,7));
    wp.push_back(make_pair(5.5,5.5));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t11 - t22
    wp.push_back(make_pair(7,8.5));
    wp.push_back(make_pair(8,8.5));
    wp.push_back(make_pair(8,10.5));
    wp.push_back(make_pair(8,12.5));
    wp.push_back(make_pair(8,15));
    wp.push_back(make_pair(8,17));
    wp.push_back(make_pair(6,19));
    wp.push_back(make_pair(4,19));
    wp.push_back(make_pair(2,19));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t11 - t31
    wp.push_back(make_pair(7,8.5));
    wp.push_back(make_pair(8.5,7));
    wp.push_back(make_pair(10.5,7));
    wp.push_back(make_pair(12.5,7));
    wp.push_back(make_pair(14.5,7));
    wp.push_back(make_pair(16,8));
    wp.push_back(make_pair(18,10));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t11 - t32
    wp.push_back(make_pair(7,8.5));
    wp.push_back(make_pair(5,8.5));
    wp.push_back(make_pair(3.5,7.5));
    wp.push_back(make_pair(2,6));
    wp.push_back(make_pair(1,4));
    wp.push_back(make_pair(1,2.5));
    wp.push_back(make_pair(1,1));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t11 - t41
    wp.push_back(make_pair(7,8.5));
    wp.push_back(make_pair(8.5,7));
    wp.push_back(make_pair(10,5.5));
    wp.push_back(make_pair(12,5));
    wp.push_back(make_pair(14,4));
    wp.push_back(make_pair(15.5,3));
    wp.push_back(make_pair(18,3));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t11 - t42
    wp.push_back(make_pair(7,8.5));
    wp.push_back(make_pair(8,8.5));
    wp.push_back(make_pair(8,10.5));
    wp.push_back(make_pair(8,12.5));
    wp.push_back(make_pair(8,15));
    wp.push_back(make_pair(9.5,16));
    wp.push_back(make_pair(11,17));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    
    
    // t12
    // t12 - t21
    wp.push_back(make_pair(16,15));
    wp.push_back(make_pair(16,12.5));
    wp.push_back(make_pair(15.5,9.5));
    wp.push_back(make_pair(14.5,7));
    wp.push_back(make_pair(12.5,7));
    wp.push_back(make_pair(10.5,7));
    wp.push_back(make_pair(8.5,7));
    wp.push_back(make_pair(6,7));
    wp.push_back(make_pair(5.5,5.5));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t12 - t22
    wp.push_back(make_pair(16,15));
    wp.push_back(make_pair(15,17));
    wp.push_back(make_pair(13,19));
    wp.push_back(make_pair(11,19));
    wp.push_back(make_pair(8.5,19));
    wp.push_back(make_pair(6,19));
    wp.push_back(make_pair(4,19));
    wp.push_back(make_pair(2,19));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t12 - t31
    wp.push_back(make_pair(16,15));
    wp.push_back(make_pair(17.5,12.5));
    wp.push_back(make_pair(18,10));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t12 - t32
    wp.push_back(make_pair(16,15));
    wp.push_back(make_pair(16,12.5));
    wp.push_back(make_pair(16,10.5));
    wp.push_back(make_pair(16,8));
    wp.push_back(make_pair(14,6));
    wp.push_back(make_pair(12,5));
    wp.push_back(make_pair(10,4));
    wp.push_back(make_pair(8,3));
    wp.push_back(make_pair(6,2));
    wp.push_back(make_pair(4,1.5));
    wp.push_back(make_pair(2,1));
    wp.push_back(make_pair(1,1));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t12 - t41
    wp.push_back(make_pair(16,15));
    wp.push_back(make_pair(16,12.5));
    wp.push_back(make_pair(16,10.5));
    wp.push_back(make_pair(16,8));
    wp.push_back(make_pair(17,5));
    wp.push_back(make_pair(18,3));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t12 - t42
    wp.push_back(make_pair(16,15));
    wp.push_back(make_pair(15,17));
    wp.push_back(make_pair(13,17));
    wp.push_back(make_pair(11,17));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    
    
    
    // t21
    // t21 - t22
    wp.push_back(make_pair(5.5,5.5));
    wp.push_back(make_pair(3.5,7.5));
    wp.push_back(make_pair(1.5,9.5));
    wp.push_back(make_pair(1,11.5));
    wp.push_back(make_pair(1,13.5));
    wp.push_back(make_pair(1,15.5));
    wp.push_back(make_pair(1,17.5));
    wp.push_back(make_pair(2,19));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t21 - t31
    wp.push_back(make_pair(5.5,5.5));
    wp.push_back(make_pair(7.5,5.5));
    wp.push_back(make_pair(10,5.5));
    wp.push_back(make_pair(12,6));
    wp.push_back(make_pair(14.5,7));
    wp.push_back(make_pair(16,8));
    wp.push_back(make_pair(18,10));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t21 - t32
    wp.push_back(make_pair(5.5,5.5));
    wp.push_back(make_pair(4,4));
    wp.push_back(make_pair(2.5,2.5));
    wp.push_back(make_pair(1,1));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t21 - t41
    wp.push_back(make_pair(5.5,5.5));
    wp.push_back(make_pair(7.5,5.5));
    wp.push_back(make_pair(10,5));
    wp.push_back(make_pair(12,4.5));
    wp.push_back(make_pair(14,4));
    wp.push_back(make_pair(15.5,3.5));
    wp.push_back(make_pair(18,3));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t21 - t42
    wp.push_back(make_pair(5.5,5.5));
    wp.push_back(make_pair(5,8.5));
    wp.push_back(make_pair(6,11));
    wp.push_back(make_pair(8,12.5));
    wp.push_back(make_pair(8,15));
    wp.push_back(make_pair(9.5,16));
    wp.push_back(make_pair(11,17));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    
    
    // t22
    // t22 - t31
    wp.push_back(make_pair(2,19));
    wp.push_back(make_pair(4,19));
    wp.push_back(make_pair(6,19));
    wp.push_back(make_pair(8.5,19));
    wp.push_back(make_pair(11,19));
    wp.push_back(make_pair(13,19));
    wp.push_back(make_pair(15,19));
    wp.push_back(make_pair(17,19));
    wp.push_back(make_pair(18,17));
    wp.push_back(make_pair(18,15));
    wp.push_back(make_pair(18,12.5));
    wp.push_back(make_pair(18,10));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t22 - t32
    wp.push_back(make_pair(2,19));
    wp.push_back(make_pair(1,17.5));
    wp.push_back(make_pair(1,15.5));
    wp.push_back(make_pair(1,13.5));
    wp.push_back(make_pair(1,11.5));
    wp.push_back(make_pair(1,9.5));
    wp.push_back(make_pair(1,7.5));
    wp.push_back(make_pair(1,5.5));
    wp.push_back(make_pair(1,3.5));
    wp.push_back(make_pair(1,1));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t22 - t41
    wp.push_back(make_pair(2,19));
    wp.push_back(make_pair(1,17.5));
    wp.push_back(make_pair(1,15.5));
    wp.push_back(make_pair(1,13.5));
    wp.push_back(make_pair(1,11.5));
    wp.push_back(make_pair(1,9.5));
    wp.push_back(make_pair(1,7.5));
    wp.push_back(make_pair(1,5.5));
    wp.push_back(make_pair(3,3));
    wp.push_back(make_pair(5.5,3));
    wp.push_back(make_pair(8,3));
    wp.push_back(make_pair(9.5,3));
    wp.push_back(make_pair(11.5,3));
    wp.push_back(make_pair(13.5,3));
    wp.push_back(make_pair(15.5,3));
    wp.push_back(make_pair(18,3));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t22 - t42
    wp.push_back(make_pair(2,19));
    wp.push_back(make_pair(4,18));
    wp.push_back(make_pair(7.5,17.5));
    wp.push_back(make_pair(11,17));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    
    
    // t31
    // t31 - t32
    wp.push_back(make_pair(18,10));
    wp.push_back(make_pair(16,8));
    wp.push_back(make_pair(14,6));
    wp.push_back(make_pair(12,5));
    wp.push_back(make_pair(10,4));
    wp.push_back(make_pair(8,3));
    wp.push_back(make_pair(6,2));
    wp.push_back(make_pair(4,1.5));
    wp.push_back(make_pair(2,1));
    wp.push_back(make_pair(1,1));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t31 - t41
    wp.push_back(make_pair(18,10));
    wp.push_back(make_pair(18,7.5));
    wp.push_back(make_pair(18,5));
    wp.push_back(make_pair(18,3));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t31 - t42
    wp.push_back(make_pair(18,10));
    wp.push_back(make_pair(16,12.5));
    wp.push_back(make_pair(14.5,14));
    wp.push_back(make_pair(13,16));
    wp.push_back(make_pair(11,17));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    
    
    // t32
    // t32 - t41
    wp.push_back(make_pair(1,1));
    wp.push_back(make_pair(3,3));
    wp.push_back(make_pair(5.5,3));
    wp.push_back(make_pair(8,3));
    wp.push_back(make_pair(9.5,3));
    wp.push_back(make_pair(11.5,3));
    wp.push_back(make_pair(13.5,3));
    wp.push_back(make_pair(15.5,3));
    wp.push_back(make_pair(18,3));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    // t32 - t42
    wp.push_back(make_pair(1,1));
    wp.push_back(make_pair(2,4));
    wp.push_back(make_pair(3,6.5));
    wp.push_back(make_pair(4.5,9));
    wp.push_back(make_pair(6,11));
    wp.push_back(make_pair(8,12.5));
    wp.push_back(make_pair(8.5,14.5));
    wp.push_back(make_pair(9.5,16));
    wp.push_back(make_pair(11,17));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);
    
    
    
    // t41
    // t41 - t42
    wp.push_back(make_pair(18,3));
    wp.push_back(make_pair(17,5));
    wp.push_back(make_pair(15.5,9.5));
    wp.push_back(make_pair(15,12));
    wp.push_back(make_pair(14.5,14));
    wp.push_back(make_pair(13,16));
    wp.push_back(make_pair(11,17));
    elem = constructMap(wp.front(), wp.back(), wp);   
    GlobMap.push_back(elem);

    
  
  
    return 0;
}