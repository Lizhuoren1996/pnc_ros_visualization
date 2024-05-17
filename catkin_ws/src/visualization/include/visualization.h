#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "MsgViewTrajectory.hpp"
#include "MsgViewParkinglotList.hpp"
#include "MsgViewObstacleList.hpp"
#include "MsgViewReflineList.hpp"
#include "MsgFusionMap.hpp"

namespace planning {

class Visualization {

public:
    Visualization(const ros::NodeHandle &nh);
    ~Visualization() = default;

    void RunOnce();
    void PublishToRviz_StartSpot(const MsgViewPoint &view_startspot);
    void PublishToRviz_ReflineList_lines(const MsgViewReflineList&view_reflinelist);
    void PublishToRviz_Trajectory_line(const MsgViewTrajectory&view_trajectory);
    void PublishToRviz_Trajectory_points(const MsgViewTrajectory&view_trajectory);
    void PublishToRviz_EgoCar(const MsgViewTrajectory&view_trajectory);
    void PublishToRviz_ParkingLots(const MsgViewParkinglotList &view_parkinglotlist);
    void PublishToRviz_ObstacleTrajectory(const MsgViewObstacleList&view_obstaclelist);
    void PublishToRviz_ObstacleShape(const MsgViewObstacleList&view_obstaclelist);
    void PublishToRviz_Refline(const MsgViewRefline &view_refline);
    void PublishToRviz_ObstacleFootprint(const MsgViewObstacleList&view_obstaclelist); 
    void GetGridMap(nav_msgs::OccupancyGrid &gmap,const MsgFusionMap&inmap, const double resolution,
          const int rows, const int cols, const int ego_row, const int ego_col);
    void PublishToRviz_GridMap(const MsgFusionMap&fusion_map);
    // void TransformToFrontAxleCenter(const MsgViewObstacleList& view_obstaclelist);

private:
    void InitPublisher();
private:
    /* data */
    ros::Publisher visualization_carboxes_publisher_;
    ros::Publisher visualized_trajectory_publisher_;
    ros::Publisher visualized_valid_trajectories_publisher_;
    ros::Publisher visualized_reference_lines_publisher_;  //multiple lines
    ros::Publisher visualized_refline_publisher_;  //one line
    ros::Publisher visualized_traffic_light_box_publisher_;
    ros::Publisher visualized_obstacle_trajectory_publisher_;
    ros::Publisher visualized_obstacle_footprint_publisher_;
    ros::Publisher visualized_obstacle_info_publisher_;
    ros::Publisher visualized_ego_vehicle_publisher_;
    ros::Publisher visualized_parkinglots_publisher_;
    ros::Publisher visualized_start_spot_publisher_;

    ros::Publisher visualized_occupancy_grid_map_publisher_; 

    ros::NodeHandle nh_;

};






} // namespace planning 



#endif
