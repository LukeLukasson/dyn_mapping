#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

// Luke
#include <iostream>
#include "../Eigen/Dense"
#include <nav_msgs/OccupancyGrid.h>

namespace scitos_2d_navigation
{

class GridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  GridLayer();

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();
  
private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  // Luke
  void matrixToMap(Eigen::MatrixXf matrix);
  
  // make the matrix available
  Eigen::MatrixXf kanon;
  
  // static map
  nav_msgs::OccupancyGrid::ConstPtr staticMap;
  
  // create publishers
  ros::Publisher staticMapPub;
  
  // initialize maps by subscribing to original map 
  //~ void initStaticMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &mainMap);
  //~ ros::Subscriber mainMapSub();
};
}
#endif
