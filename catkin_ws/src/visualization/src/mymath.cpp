#include "MsgViewTrajectory.hpp"
#include "MsgViewRefline.hpp"
#include <cmath>
#include "mymath.h"

// get relative location based on orientation and distance
geometry_msgs::Point GetRelativeLocation_Egocar(const MsgViewTrajectoryPoint p, const double l, const double theta){
    geometry_msgs::Point pr;
    pr.x = p.x + l*cos(theta);
    pr.y = p.y + l*sin(theta);
    return pr;
}

geometry_msgs::Point GetRelativeLocation_Obs(const MsgViewObstacleTrajectoryPoint p, const double l, const double theta){
    geometry_msgs::Point pr;
    pr.x = p.x + l*cos(theta);
    pr.y = p.y + l*sin(theta);
    pr.z = 0;
    return pr;
}

geometry_msgs::Point GetRelativeLocation_StartSpot(const MsgViewPoint p, const double l, const double theta){
    geometry_msgs::Point pr;
    pr.x = p.x + l*cos(theta);
    pr.y = p.y + l*sin(theta);
    pr.z = 0;
    return pr;
}

// get vertex of rectangle based on center point
std::vector<geometry_msgs::Point> GetRectangleVertex(const MsgViewObstacleTrajectoryPoint center_p, const double theta, const double l, const double w){
   std::vector<geometry_msgs::Point> vertex;
    double half_diagonal = 0.5*sqrt(l*l+w*w); 
    double alpha = atan2(w,l);
    double deg[4] = {theta+alpha,theta-alpha,theta+alpha-M_PI,theta-alpha-M_PI}; //clockwise
    for (size_t i = 0; i < 4; i++)
    {  
        geometry_msgs::Point p;
        p = GetRelativeLocation_Obs(center_p,half_diagonal,deg[i]);
        vertex.push_back(p);
    }
    return vertex;
}

// get vehicle 2D box based on front axle center point
std::vector<geometry_msgs::Point> Get2DBoxVertex_Egocar(const MsgViewTrajectoryPoint center_p, const double theta, const double l, const double w,const double front_hang){
    double a = front_hang;
    std::vector<geometry_msgs::Point> vertex;
    double dist1 = sqrt(a*a+0.25*w*w); double dist2 = sqrt(0.25*w*w+(l-a)*(l-a));
    double alpha = atan2(0.5*w,a); double beta = atan2(0.5*w,(l-a));
    
    double dists[4] = {dist1,dist1,dist2,dist2}; //clockwise
    double degs[4] = {theta+alpha,theta-alpha,theta+beta-M_PI,theta-beta-M_PI}; //clockwise
    if (-M_PI<theta<0)
    {
        double dists[4] = {dist1,dist1,dist2,dist2}; //clockwise
        double degs[4] = {theta-alpha,theta+alpha,theta-beta-M_PI,theta+beta-M_PI}; //clockwise
    }
    
    for (size_t i = 0; i < 4; i++)
    {  
        geometry_msgs::Point p;
        p = GetRelativeLocation_Egocar(center_p,dists[i],degs[i]);
        vertex.push_back(p);
    }
    return vertex;
}

std::vector<geometry_msgs::Point> Get2DBoxVertex_Obs(const MsgViewObstacleTrajectoryPoint center_p, 
                 const double theta, const double l, const double w,const double front_hang){
    double a = front_hang;
    std::vector<geometry_msgs::Point> vertex;
    double dist1 = sqrt(a*a+0.25*w*w); double dist2 = sqrt(0.25*w*w+(l-a)*(l-a));
    double alpha = atan2(0.5*w,a); double beta = atan2(0.5*w,(l-a));
    
    double dists[4] = {dist1,dist1,dist2,dist2}; //clockwise
    double degs[4] = {theta+alpha,theta-alpha,theta+beta-M_PI,theta-beta-M_PI}; //clockwise
    if (-M_PI<theta<0)
    {
        double dists[4] = {dist1,dist1,dist2,dist2}; //clockwise
        double degs[4] = {theta-alpha,theta+alpha,theta-beta-M_PI,theta+beta-M_PI}; //clockwise
    }
    
    for (size_t i = 0; i < 4; i++)
    {  
        geometry_msgs::Point p;
        p = GetRelativeLocation_Obs(center_p,dists[i],degs[i]);
        vertex.push_back(p);
    }
    return vertex;
}

// get vehicle 2D box based on front axle center point
std::vector<geometry_msgs::Point> Get2DBoxVertex_StartSpot(const MsgViewPoint center_p, const double theta, const double l, const double w,const double front_hang){
    double a = front_hang;
    std::vector<geometry_msgs::Point> vertex;
    double dist1 = sqrt(a*a+0.25*w*w); double dist2 = sqrt(0.25*w*w+(l-a)*(l-a));
    double alpha = atan2(0.5*w,a); double beta = atan2(0.5*w,(l-a));
    
    double dists[4] = {dist1,dist1,dist2,dist2}; //clockwise
    double degs[4] = {theta+alpha,theta-alpha,theta+beta-M_PI,theta-beta-M_PI}; //clockwise
    if (-M_PI<theta<0)
    {
        double dists[4] = {dist1,dist1,dist2,dist2}; //clockwise
        double degs[4] = {theta-alpha,theta+alpha,theta-beta-M_PI,theta+beta-M_PI}; //clockwise
    }
    
    for (size_t i = 0; i < 4; i++)
    {  
        geometry_msgs::Point p;
        p = GetRelativeLocation_StartSpot(center_p,dists[i],degs[i]);
        vertex.push_back(p);
    }
    return vertex;
}

// get vehicle 3D box based on vehicle 2D box
std::vector<geometry_msgs::Point> Get3DBoxVertex(const MsgViewObstacleTrajectoryPoint center_p, 
                                                 const double theta, const double l, 
                                                const double w,const double front_hang,const double h){
    std::vector<geometry_msgs::Point> vertex2D;
    //get the front axle center point based on 2Dbox_center
    MsgViewObstacleTrajectoryPoint front_center; geometry_msgs::Point p;
    p = GetRelativeLocation_Obs(center_p,0.5*l-front_hang,theta);
    front_center.x = p.x; front_center.y = p.y;
    vertex2D = Get2DBoxVertex_Obs(front_center,theta,l,w,front_hang);
    std::vector<geometry_msgs::Point> vertex3D;
    for (size_t i = 0; i < 4; i++)
    {
        vertex3D.push_back(vertex2D[i]);
    } 
    
    for (size_t i = 0; i < 4; i++)
    {
        geometry_msgs::Point ph;
        ph = vertex2D[i];
        ph.z = ph.z+h;
        vertex3D.push_back(ph);
    }    
    return vertex3D;
}

// get vehicle 3D box based on vehicle 2D box
std::vector<geometry_msgs::Point> Get3DBoxVertex_StartSpot(const MsgViewPoint center_p, 
                                                 const double theta, const double l, 
                                                const double w,const double front_hang,const double h){
    std::vector<geometry_msgs::Point> vertex2D;
    vertex2D = Get2DBoxVertex_StartSpot(center_p,theta,l,w,front_hang);
    std::vector<geometry_msgs::Point> vertex3D;
    for (size_t i = 0; i < 4; i++)
    {
        vertex3D.push_back(vertex2D[i]);
    } 
    
    for (size_t i = 0; i < 4; i++)
    {
        geometry_msgs::Point ph;
        ph = vertex2D[i];
        ph.z = ph.z+h;
        vertex3D.push_back(ph);
    }    
    return vertex3D;
}