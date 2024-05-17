#include "visualization.h"
#include "zcm_message_manager.h"
#include "MsgViewTrajectory.hpp"
#include "MsgViewRefline.hpp"
#include "MsgViewParkinglotList.hpp"
#include "mymath.h"

namespace planning {
// visualization publisher
Visualization::Visualization(const ros::NodeHandle &nh) {
    nh_ = nh; 
    InitPublisher();
}

void Visualization::InitPublisher() {

    this->visualized_start_spot_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      "/motion_planner/visualized_start_spot", 1);
    this->visualized_trajectory_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      "/motion_planner/visualized_trajectory", 1);
    this->visualized_valid_trajectories_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/motion_planner/visualized_valid_trajectories", 1);
    this->visualized_reference_lines_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/motion_planner/visualized_reference_lines", 1);
    this->visualized_refline_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      "/motion_planner/visualized_refline", 1);
    this->visualized_traffic_light_box_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/motion_planner/visualized_traffic_light_boxes", 1);
    this->visualized_obstacle_trajectory_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/motion_planner/visualized_obstacle_trajectries", 1);
    this->visualized_obstacle_footprint_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/motion_planner/visualized_obstacle_footprint", 1);
    this->visualized_obstacle_info_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/motion_planner/visualized_obstacle_infos", 1);
    this->visualized_ego_vehicle_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("/motion_planner/visualized_ego_vehicle", 1);
    this->visualized_parkinglots_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("/motion_planner/visualized_parkinglots", 1);
    this->visualized_occupancy_grid_map_publisher_=nh_.advertise<nav_msgs::OccupancyGrid>(
      "/motion_planner/visualized_occupancy_grid_map",1);
    

}

// get GridMap 
void Visualization::GetGridMap(nav_msgs::OccupancyGrid &gmap,const MsgFusionMap&inmap, const double resolution,
          const int rows, const int cols, const int ego_row, const int ego_col){
    int N_square; // number of cells in a row(square grid map)
    int dN;
    if (rows>cols) { N_square = rows; dN = (rows-cols)/2;}
    else { N_square = cols; dN = (cols-rows)/2; }

    gmap.header.frame_id = "map"; gmap.header.stamp = ros::Time::now();
    gmap.info.resolution = resolution;
    gmap.info.width = N_square; gmap.info.height = N_square;

    double dx = (rows-ego_row)*resolution+0.5*resolution;
    double dy = (cols-1)*resolution+0.5*resolution;
    gmap.info.origin.position.x = -dx; gmap.info.origin.position.y = dy; gmap.info.origin.position.z = 0;
    gmap.info.origin.orientation.w = 0.707;
    gmap.info.origin.orientation.x = 0;  gmap.info.origin.orientation.y = 0; gmap.info.origin.orientation.z = -0.707;
    
    for (size_t i = 0; i < N_square; i++)
    {
        for (size_t j = 0; j < N_square; j++)
        {
            int8_t a; int jj = j-dN;
            if (j<dN || j>=(N_square-dN)){
                //  if(j==0||i==0){a=127;gmap.data.push_back(a);}else{
                  a = -1; gmap.data.push_back(a);} //unknown
            else{
               uint8_t value = inmap.map_cells[rows-i-1][j+dN];              
               value = (int)value;
               if(value==0) { a = 0; gmap.data.push_back(a); } //free
               else{ a = 100; gmap.data.push_back(a); } //static-obstacle
            }
        }   
    }
        
}

// OccupancyGrid
void Visualization::PublishToRviz_GridMap(const MsgFusionMap&fusion_map){
  if(fusion_map.map_cells.empty()){return;}
  nav_msgs::OccupancyGrid gmap;
  int rows = fusion_map.map_row_num; int cols = fusion_map.map_column_num; 
  int ego_row = fusion_map.car_center_row; int ego_col = fusion_map.car_center_column;
  double resolution = fusion_map.map_resolution;
  
  GetGridMap(gmap,fusion_map,resolution,rows,cols,ego_row,ego_col);

  this->visualized_occupancy_grid_map_publisher_.publish(gmap);
}


// start spot
void Visualization::PublishToRviz_StartSpot(const MsgViewPoint &view_startspot){
  
  visualization_msgs::Marker start_spot;
  start_spot.header.frame_id = "map";
  start_spot.ns = "start_spot";
  start_spot.id = 0;
  start_spot.header.stamp = ros::Time::now();
  start_spot.type = visualization_msgs::Marker::LINE_LIST;
  start_spot.scale.x = 0.2;
  start_spot.color.a = 1.0;
  start_spot.color.r = 0; start_spot.color.g = 1; start_spot.color.b = 0;
  start_spot.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point sp;
  sp.x = view_startspot.x;  sp.y = view_startspot.y;  

  double l = 4.0; double w = 2.0; double a = 0.8; double h = 1.0; // parameters needed!!!
 
      std::vector<geometry_msgs::Point> vertex3D;
      vertex3D = Get3DBoxVertex_StartSpot(view_startspot,view_startspot.theta,l,w,a,h);
      for (size_t k = 0; k < 3; k++)
      {
        start_spot.points.push_back(vertex3D[k]); start_spot.points.push_back(vertex3D[k+1]);  
      }
      start_spot.points.push_back(vertex3D[3]);  start_spot.points.push_back(vertex3D[0]);
      for (size_t kk = 4; kk < 7; kk++)
      {
        start_spot.points.push_back(vertex3D[kk]); start_spot.points.push_back(vertex3D[kk+1]);
      }
      start_spot.points.push_back(vertex3D[7]);  start_spot.points.push_back(vertex3D[4]);
      for (size_t kkk = 0; kkk < 4; kkk++)
      {
        start_spot.points.push_back(vertex3D[kkk]); start_spot.points.push_back(vertex3D[kkk+4]);
      }

  this->visualized_start_spot_publisher_.publish(start_spot);
}

// refline
void Visualization::PublishToRviz_Refline(const MsgViewRefline &view_refline){
  if (view_refline.poses.empty())
  {   return;  }
  
  visualization_msgs::Marker refline;
  refline.header.frame_id = "map";
  refline.header.stamp = ros::Time::now();
  refline.id = 0;
  refline.ns = "refline";
  refline.scale.x = 0.15;
  refline.color.a = 1.0;
  refline.color.r = 0.11765;   refline.color.g = 0.56471;   refline.color.b = 1.0;
  refline.type = visualization_msgs::Marker::LINE_STRIP;
  refline.action = visualization_msgs::Marker::ADD;
  refline.lifetime = ros::Duration(0.1001);


  for (size_t i = 0; i < view_refline.num_poses; i++)
  {
    geometry_msgs::Point rp;
    rp.x = view_refline.poses[i].x;
    rp.y = view_refline.poses[i].y;
    refline.points.push_back(rp);
  }
  this->visualized_refline_publisher_.publish(refline);
}

// reflines
void Visualization::PublishToRviz_ReflineList_lines(const MsgViewReflineList &view_reflinelist){
  if (view_reflinelist.reflines.empty()) {return;}
  visualization_msgs::MarkerArray reflines;
  int32_t num = view_reflinelist.num;
  // std::cout << "num: " << num << std::endl;
  for (size_t i = 0; i < num; i++)
  {
    visualization_msgs::Marker ref_line;
    ref_line.header.frame_id  = "map";
    ref_line.header.stamp  = ros::Time::now();
    ref_line.id = i; 
    ref_line.type = visualization_msgs::Marker::LINE_STRIP; 
    ref_line.scale.x = 0.1;
    ref_line.ns = "reflines_and_points";
    ref_line.color.a = 1.0; 
    ref_line.color.r = 1.0;
    ref_line.color.g = 1.0;
    ref_line.color.b = 1.0;
    ref_line.action = visualization_msgs::Marker::ADD;

    for (size_t j = 0; j < view_reflinelist.reflines[i].num_poses; j++)
    {
      geometry_msgs::Point rp;
      rp.x = view_reflinelist.reflines[i].poses[j].x;
      rp.y = view_reflinelist.reflines[i].poses[j].y;
      ref_line.points.push_back(rp);
    }
    reflines.markers.push_back(ref_line);
  }
  this->visualized_reference_lines_publisher_.publish(reflines);
}

//  trajectory points
void Visualization::PublishToRviz_Trajectory_points(const MsgViewTrajectory&view_trajectory){
  if (view_trajectory.poses.empty()) {return;}
  
  visualization_msgs::Marker trajectory_points; 
  trajectory_points.header.frame_id = "map";
  trajectory_points.ns = "points_and_lines";
  trajectory_points.id = 0;
  trajectory_points.header.stamp = ros::Time::now();
  trajectory_points.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_points.scale.x = 0.05;
  // trajectory_points.scale.y = 0.1;
  trajectory_points.color.a = 1.0;
  trajectory_points.color.g = 1.0;
  trajectory_points.action = visualization_msgs::Marker::ADD;

  for(size_t i = 0; i<view_trajectory.num_poses; i++){
    geometry_msgs::Point p;
    p.x = view_trajectory.poses[i].x;
    p.y = view_trajectory.poses[i].y;
    trajectory_points.points.push_back(p);
  }
 
  this->visualized_trajectory_publisher_.publish(trajectory_points);
}

// trajectory line
void Visualization::PublishToRviz_Trajectory_line(const MsgViewTrajectory&view_trajectory){
  if (view_trajectory.poses.empty()) {return;}
  
  visualization_msgs::Marker trajectory_points; 
  trajectory_points.header.frame_id = "map";
  trajectory_points.ns = "points_and_lines";
  trajectory_points.id = 0;
  trajectory_points.header.stamp = ros::Time::now();
  trajectory_points.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_points.scale.x = 0.1;
  trajectory_points.color.a = 1.0;
  trajectory_points.color.r = 0.41961; trajectory_points.color.g = 0.55686; trajectory_points.color.b = 0.13725;
  trajectory_points.action = visualization_msgs::Marker::ADD;

  for(size_t i = 0; i<view_trajectory.num_poses; i++){
    geometry_msgs::Point p;
    p.x = view_trajectory.poses[i].x;
    p.y = view_trajectory.poses[i].y;
    trajectory_points.points.push_back(p);
  }
 
  this->visualized_trajectory_publisher_.publish(trajectory_points);
}

// obstacle_trajectory
void Visualization::PublishToRviz_ObstacleTrajectory(const MsgViewObstacleList&view_obstaclelist){
  if (view_obstaclelist.objects.empty()) {return;}
  
  visualization_msgs::MarkerArray obs_trajectories; 

  for (size_t i = 0; i < view_obstaclelist.num; i++)
  {
    visualization_msgs::Marker obs_trajectory;
    obs_trajectory.header.frame_id = "map";
    obs_trajectory.ns = "obstacle_trajectory";
    obs_trajectory.id = i;
    obs_trajectory.header.stamp = ros::Time::now();
    obs_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    obs_trajectory.scale.x = 0.1;
    obs_trajectory.color.a = 1.0;
    obs_trajectory.color.r = 1.0; obs_trajectory.color.g = 0.07843; obs_trajectory.color.b = 0.57647;
    obs_trajectory.action = visualization_msgs::Marker::ADD;
    obs_trajectory.lifetime = ros::Duration(0.1001);

    for (size_t j = 0; j < view_obstaclelist.objects[i].num_poses; j++)
    {
      geometry_msgs::Point otp;
      otp.x = view_obstaclelist.objects[i].poses[j].x;
      otp.y = view_obstaclelist.objects[i].poses[j].y;
      obs_trajectory.points.push_back(otp);
    }
    obs_trajectories.markers.push_back(obs_trajectory);  
  }

  this->visualized_obstacle_trajectory_publisher_.publish(obs_trajectories);
}

// obstacles_footprint
void Visualization::PublishToRviz_ObstacleFootprint(const MsgViewObstacleList&view_obstaclelist){
  if (view_obstaclelist.objects.empty()) {return;}
  
  visualization_msgs::MarkerArray obs_trajectories; 
  for (size_t i = 0; i < view_obstaclelist.num; i++)
  {
    visualization_msgs::Marker obs_trajectory;
    obs_trajectory.header.frame_id = "map";
    obs_trajectory.ns = "obstacle_footprint";
    obs_trajectory.id = i;
    obs_trajectory.header.stamp = ros::Time::now();
    obs_trajectory.type = visualization_msgs::Marker::LINE_LIST;
    obs_trajectory.scale.x = 0.1;
    obs_trajectory.color.a = 1.0;
    obs_trajectory.color.r = 1.0; obs_trajectory.color.g = 0.07843; obs_trajectory.color.b = 0.57647;
    obs_trajectory.action = visualization_msgs::Marker::ADD;
    obs_trajectory.lifetime = ros::Duration(0.1001);

    for (size_t j = 0; j < view_obstaclelist.objects[i].num_poses; j++)
    {
      std::vector<geometry_msgs::Point> vertex;
      double dx = view_obstaclelist.objects[i].poses[3].x-view_obstaclelist.objects[i].poses.front().x;
      double dy = view_obstaclelist.objects[i].poses[3].y-view_obstaclelist.objects[i].poses.front().y;
      double traj_heading = view_obstaclelist.objects[i].poses[j].theta;
      // if(dx==0){ 
      //   if(dy>0) traj_heading = M_PI*0.5;
      //   else traj_heading = -M_PI*0.5;
      // }else{
      //   traj_heading = atan2(dy,dx);
      // } 
      vertex = GetRectangleVertex(view_obstaclelist.objects[i].poses[j],traj_heading,
      view_obstaclelist.objects[i].length,view_obstaclelist.objects[i].width);
      for (size_t i = 0; i < vertex.size()-1; i++)
      {
        obs_trajectory.points.push_back(vertex[i]);
        obs_trajectory.points.push_back(vertex[i+1]);
      }
      obs_trajectory.points.push_back(vertex.back());
      obs_trajectory.points.push_back(vertex[0]);     

    }
    obs_trajectories.markers.push_back(obs_trajectory);  
  }
  this->visualized_obstacle_footprint_publisher_.publish(obs_trajectories);
}

// obstacle shape
void Visualization::PublishToRviz_ObstacleShape(const MsgViewObstacleList&view_obstaclelist){
  if (view_obstaclelist.objects.empty()) {return;}

  visualization_msgs::MarkerArray obstacles;
  int32_t num = view_obstaclelist.num;
  double h = 1.0; double a = 0.8; 
  for (size_t i = 0; i < num; i++)
  {
    visualization_msgs::Marker obs;
    obs.header.frame_id = "map";
    obs.header.stamp = ros::Time::now();
    obs.ns = "obstacle_shape";
    obs.id = i;
    obs.type = visualization_msgs::Marker::LINE_LIST;
    obs.scale.x = 0.2;
    obs.color.a = 1.0;
    obs.color.r = 1.0; obs.color.g = 0.07843; obs.color.b = 0.57647;
    obs.action = visualization_msgs::Marker::ADD;
    obs.lifetime = ros::Duration(0.1001);

    for (size_t j = 0; j < 1; j++)
    {
      double dx = view_obstaclelist.objects[i].poses[3].x-view_obstaclelist.objects[i].poses.front().x;
      double dy = view_obstaclelist.objects[i].poses[3].y-view_obstaclelist.objects[i].poses.front().y;
      double traj_heading;
      if(dx==0){ 
        if(dy>0) traj_heading = M_PI*0.5;
        else traj_heading = -M_PI*0.5;
      }else{
        traj_heading = atan2(dy,dx);
      } 
       
      std::vector<geometry_msgs::Point> vertex3D;
      vertex3D = Get3DBoxVertex(view_obstaclelist.objects[i].poses[j],traj_heading,
      view_obstaclelist.objects[i].length,view_obstaclelist.objects[i].width,a,h);
      for (size_t k = 0; k < 3; k++)
      {
        obs.points.push_back(vertex3D[k]); obs.points.push_back(vertex3D[k+1]);  
      }
      obs.points.push_back(vertex3D[3]);  obs.points.push_back(vertex3D[0]);
      for (size_t kk = 4; kk < 7; kk++)
      {
        obs.points.push_back(vertex3D[kk]); obs.points.push_back(vertex3D[kk+1]);
      }
      obs.points.push_back(vertex3D[7]);  obs.points.push_back(vertex3D[4]);
      for (size_t kkk = 0; kkk < 4; kkk++)
      {
        obs.points.push_back(vertex3D[kkk]); obs.points.push_back(vertex3D[kkk+4]);
      }
    }
    obstacles.markers.push_back(obs);
  }
  this->visualized_obstacle_info_publisher_.publish(obstacles); 

}

// parkinglots
void Visualization::PublishToRviz_ParkingLots(const MsgViewParkinglotList &view_parkinglotlist){
  if (view_parkinglotlist.parking_lots.empty())
  {  return;  }
  
  visualization_msgs::Marker parkinglotlist; 
  parkinglotlist.header.frame_id = "map";
  parkinglotlist.ns = "parkinglots";
  parkinglotlist.id = 0;
  parkinglotlist.header.stamp = ros::Time::now();
  parkinglotlist.type = visualization_msgs::Marker::LINE_LIST;
  parkinglotlist.scale.x = 0.1;
  parkinglotlist.color.a = 1.0;
  parkinglotlist.color.g = 1.0;
  parkinglotlist.color.b = 1.0;
  parkinglotlist.color.r = 1.0;
  parkinglotlist.action = visualization_msgs::Marker::ADD;
  // std::cout << "view_parkinglotlist.num: "  << view_parkinglotlist.num << std::endl;
  for (size_t i = 0; i < view_parkinglotlist.num; i++)
  {
      geometry_msgs::Point p[4];
      p[0].x = view_parkinglotlist.parking_lots[i].p0.x;
      p[0].y = view_parkinglotlist.parking_lots[i].p0.y;
      p[1].x = view_parkinglotlist.parking_lots[i].p1.x;
      p[1].y = view_parkinglotlist.parking_lots[i].p1.y;
      p[2].x = view_parkinglotlist.parking_lots[i].p2.x;
      p[2].y = view_parkinglotlist.parking_lots[i].p2.y;
      p[3].x = view_parkinglotlist.parking_lots[i].p3.x;
      p[3].y = view_parkinglotlist.parking_lots[i].p3.y;
      
      for (size_t j = 0; j < 3; j++)
      {
        parkinglotlist.points.push_back(p[j]);
        parkinglotlist.points.push_back(p[j+1]);
      }
      parkinglotlist.points.push_back(p[3]);
      parkinglotlist.points.push_back(p[0]);   
  }
  this->visualized_parkinglots_publisher_.publish(parkinglotlist);
}

// egocar
void Visualization::PublishToRviz_EgoCar(const MsgViewTrajectory&view_trajectory){
  if (view_trajectory.poses.empty())
  { return; }
  
  visualization_msgs::Marker carbox; 
  carbox.header.frame_id = "map";
  carbox.ns = "egocar";
  carbox.id = 0;
  carbox.header.stamp = ros::Time::now();
  carbox.type = visualization_msgs::Marker::LINE_LIST;
  carbox.scale.x = 0.1;
  carbox.color.a = 1.0;
  carbox.color.r = 0.41961; carbox.color.g = 0.55686; carbox.color.b = 0.13725;
  carbox.action = visualization_msgs::Marker::ADD;

  for (size_t j = 0; j < view_trajectory.num_poses; j++)
  {
      std::vector<geometry_msgs::Point> vertex;
      vertex = Get2DBoxVertex_Egocar(view_trajectory.poses[j],view_trajectory.poses[j].theta,4.0,2.0,0.8);
      for (size_t i = 0; i < vertex.size()-1; i++)
      {
        carbox.points.push_back(vertex[i]);
        carbox.points.push_back(vertex[i+1]);
      }
      carbox.points.push_back(vertex.back());
      carbox.points.push_back(vertex[0]);     
  }

  this->visualized_ego_vehicle_publisher_.publish(carbox);

}

// transform obstacle trajectory point into front axle center



void Visualization::RunOnce() {

    planning::ZcmMessageManager *message_manager =
    planning::ZcmMessageManager::GetInstance();
    
    MsgViewPoint start_spot;
    message_manager->GetViewStartSpot(start_spot); 

    MsgFusionMap fusion_map;
    message_manager->GetFusionMap(fusion_map);

    MsgViewRefline view_refline;
    message_manager->GetViewRefline(view_refline);

    MsgViewReflineList view_reflinelist;
    message_manager->GetViewReflinelist(view_reflinelist); 
  
    MsgViewTrajectory view_trajectory;
    bool spot_exist;
    spot_exist = message_manager->GetViewTrajectory(view_trajectory);
    if (spot_exist)
    {
      this->PublishToRviz_StartSpot(start_spot); //start_spot
    }
    

    MsgViewObstacleList view_obstaclelist;
    message_manager->GetViewObstaclelist(view_obstaclelist); 

    MsgViewParkinglotList view_parkinglotlist;
    message_manager->GetViewParkinglotlist(view_parkinglotlist); 

    
    this->PublishToRviz_Refline(view_refline); //refline
    this->PublishToRviz_ReflineList_lines(view_reflinelist); //reflines
    this->PublishToRviz_Trajectory_line(view_trajectory); //trajectory 
    this->PublishToRviz_ObstacleTrajectory(view_obstaclelist); //obstacle_trajectory
    this->PublishToRviz_ObstacleFootprint(view_obstaclelist); // obstacle_footprint
    this->PublishToRviz_ObstacleShape(view_obstaclelist); //obstacle_shape
    // this->PublishToRviz_ParkingLots(view_parkinglotlist); //parkinglots

    this->PublishToRviz_EgoCar(view_trajectory);  //egocar
    this->PublishToRviz_GridMap(fusion_map); //grid map
}



}//namespace planning