#include <unistd.h>
#include <zcm/zcm-cpp.hpp>
#include <cmath>
#include <iostream>
// #include "example_t.hpp"
#include "MsgViewTrajectory.hpp"
#include "MsgViewReflineList.hpp"
#include "MsgViewParkinglotList.hpp"
#include "MsgViewObstacleList.hpp"

int main(int argc, char *argv[])
{
    zcm::ZCM zcm {"ipc"};
    if (!zcm.good())
        return 1;

    //start point
    MsgViewPoint start_spot;
    start_spot.x = -5;  start_spot.y = 0;  start_spot.theta = 0;

    //refline
    MsgViewReflineList my_reflinelist;
    my_reflinelist.num = 3;
    for (size_t i = 0; i < my_reflinelist.num; i++)
    {
        MsgViewRefline myrefline;
        myrefline.num_poses = 100;
        for (size_t j = 0; j < myrefline.num_poses; j++)
        {
            MsgViewPoint rp;
            rp.x = 3+1*i+0.5*j;
            rp.y = -2+0.8*i+0.7*j;
            rp.theta = 0.1*M_PI*i;
            myrefline.poses.push_back(rp);
        }
        my_reflinelist.reflines.push_back(myrefline);
    }
    

    //trajectory
    MsgViewTrajectory my_trajectory;
    my_trajectory.num_poses = 100;
    for (int i = 0; i < 100; ++i) {
        MsgViewTrajectoryPoint tp;
        tp.x = i * 0.5;
        tp.y = 0;
        tp.theta = 0.;
        my_trajectory.poses.emplace_back(tp);
    }

    //parkinglots
    MsgViewParkinglotList my_parkinglotlist;
    my_parkinglotlist.num = 3;
    double l = 1.0; double w = 0.5;
    for (size_t i = 0; i < my_parkinglotlist.num; i++)
    {
        MsgViewParkinglot pl;
        MsgViewParkingPoint pp1,pp2,pp3,pp4;
        pp1.x = 0.5+i*0.6; pp1.y = 0.5+i*0.5; pl.p0=pp1;
        pp2.x = 0.5+i*0.6; pp2.y = 0.5+l+i*0.5; pl.p1=pp2;
        pp3.x = w+0.5+i*0.6; pp3.y = 0.5+l+i*0.5; pl.p2=pp3;
        pp4.x = w+0.5+i*0.6; pp4.y = 0.5+i*0.5; pl.p3=pp4;
        my_parkinglotlist.parking_lots.push_back(pl);
    }

    //obstacle
    MsgViewObstacleList my_obstaclelist;
    my_obstaclelist.num = 21;
    for (size_t i = 0; i < my_obstaclelist.num; i++)
    {
        MsgViewObstacle obs;
        obs.x = 3;
        obs.y = -2;
        obs.v = 0.1;
        obs.width = 2.5;
        obs.length = 4.0;
        obs.heading = 0.1*M_PI*i;
        obs.num_poses = 100;

        for (size_t j = 0; j < obs.num_poses; j++)
        {
           MsgViewObstacleTrajectoryPoint op;
           op.x = 3+1*i+0.5*j;
           op.y = -2+0.8*i+0.7*j;
           op.theta = 0.5*M_PI*j;
           op.vel = 10*i;
           op.relative_time = 0.1*j;
           obs.poses.push_back(op);
        }
        my_obstaclelist.objects.push_back(obs);
    }
    

    while (1) {
        zcm.publish("MsgViewTrajectorySignal", &my_trajectory);
        zcm.publish("MsgViewParkinglotListSignal", &my_parkinglotlist);
        zcm.publish("MsgViewObstacleListSignal", &my_obstaclelist);
        zcm.publish("MsgViewRefLineSignal", &my_reflinelist);
        zcm.publish("MsgViewStartSpotSignal", &start_spot);
        usleep(10*1000);
    }

    return 0;
}
