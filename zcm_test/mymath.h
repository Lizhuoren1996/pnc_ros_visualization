#ifndef MYMATH_H
#define MYMATH_H
#include "visualization.h"
#include "zcm_message_manager.h"
#include "MsgViewTrajectory.hpp"
#include "MsgViewRefline.hpp"
#include "MsgViewParkinglotList.hpp"
#include "MsgViewObstacleTrajectoryPoint.hpp"
#include <cmath>

geometry_msgs::Point GetRelativeLocation_Egocar(const MsgViewTrajectoryPoint p, const double l, const double theta);
geometry_msgs::Point GetRelativeLocation_Obs(const MsgViewObstacleTrajectoryPoint p, const double l, const double theta);
geometry_msgs::Point GetRelativeLocation_StartSpot(const MsgViewPoint p, const double l, const double theta);

std::vector<geometry_msgs::Point> GetRectangleVertex(const MsgViewTrajectoryPoint center_p, const double theta, const double l, const double w);

std::vector<geometry_msgs::Point> Get2DBoxVertex_Egocar(const MsgViewTrajectoryPoint center_p, const double theta, const double l, const double w,const double front_hang);
std::vector<geometry_msgs::Point> Get2DBoxVertex_Obs(const MsgViewObstacleTrajectoryPoint center_p, const double theta, const double l, const double w,const double front_hang);
std::vector<geometry_msgs::Point> Get2DBoxVertex_StartSpot(const MsgViewPoint center_p, const double theta, const double l, const double w,const double front_hang);

std::vector<geometry_msgs::Point> Get3DBoxVertex(const MsgViewObstacleTrajectoryPoint center_p, 
                                                 const double theta, const double l, 
                                                const double w,const double front_hang,const double h);
std::vector<geometry_msgs::Point> Get3DBoxVertex_StartSpot(const MsgViewPoint center_p, 
                                                 const double theta, const double l, 
                                                const double w,const double front_hang,const double h);


#endif