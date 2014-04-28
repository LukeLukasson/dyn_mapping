#include <scitos_2d_navigation/grid_layer.h>
#include <pluginlib/class_list_macros.h>

// debug
#include "std_msgs/String.h"
#include <vector>

PLUGINLIB_EXPORT_CLASS(scitos_2d_navigation::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

// Luke
using Eigen::MatrixXf;

namespace scitos_2d_navigation
{

// Constructor
//~ GridLayer::GridLayer(): nh("~/" + name_)
//~ {
    //~ // define publisher
    //~ staticMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/static_map", 10);
//~ 
    //~ // define subscriber
    //~ mainMapSub = nh.subscribe("/map", 2, &GridLayer::initStaticMapCallback, this);
//~ }

// Destructor
GridLayer::~GridLayer()
{
    // void
};

void GridLayer::onInitialize()
{
    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    
    // Luke
    // define publisher
    staticMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/static_map", 10);
    chatterPub = nh.advertise<std_msgs::String>("/chatter", 1000);
    // define subscriber
    //~ mainMapSub = nh.subscribe("/map", 2, &GridLayer::initStaticMapCallback, this); 
    
    // initialize an nxn matrix with any scalar
    kanon = MatrixXf::Constant(2,2,100);
    kanon(0,0) = 1.1;

    std::cout << std::endl << kanon << std::endl;

    matrixToMap(kanon);
    
    initStaticMap(staticMap);
    
}

// initialize static map
void GridLayer::initStaticMap(nav_msgs::OccupancyGrid &map)
{
    ROS_WARN("initializing static map");
    
    // handeling nav_msgs/MapMetaData
    map.info.resolution = 1.0;      // float32
    map.info.width = 2;             // uint32
    map.info.height = 2;            // uint32
    int p[] = {0, 0, 0, 0};
    std::vector<signed char> a(p, p+4);
    //~ for( std::vector<signed char>::const_iterator i = a.begin(); i != a.end(); ++i) {
        //~ std::cout << *i << ' ';
    //~ }
    map.data = a;
}

// push values of matrix to OccupancyGrid of map
void GridLayer::matrixToMap(Eigen::MatrixXf matrix)
{
    std::cout << matrix << std::endl;
    
    // debug
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world, msg";
    msg.data = ss.str();
    chatterPub.publish(msg);
    staticMapPub.publish(staticMap);
}

// init static map by waiting for the original map
void initStaticMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &mainMap)
{
    ROS_WARN("I got the map!!!");    
}


void GridLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
}

void GridLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
    if (!enabled_)
    return;

    double mark_x = origin_x + cos(origin_yaw), mark_y = origin_y + sin(origin_yaw);
    unsigned int mx;
    unsigned int my;
    if(worldToMap(mark_x, mark_y, mx, my))
    {
        setCost(mx, my, LETHAL_OBSTACLE);
    }

    ROS_INFO("-------------------------");
    ROS_INFO("-----------updateBounds--");
    ROS_INFO("origin_x:   %f", origin_x);
    ROS_INFO("origin_y:   %f", origin_y);
    ROS_INFO("origin_yaw: %f", origin_yaw);
    ROS_INFO("mark_x:     %f", mark_x);
    ROS_INFO("mark_y:     %f", mark_y);
    ROS_INFO("mx:         %i", mx);
    ROS_INFO("my:         %i", my);
    //~ ROS_INFO("*min_x:     %f", *min_x);
    //~ ROS_INFO("*min_y:     %f", *min_y); 
    //~ ROS_INFO("*max_x:     %f", *max_x);
    //~ ROS_INFO("*max_y:     %f", *max_y);
    ROS_INFO("-------------------------");

    *min_x = std::min(*min_x, mark_x);
    *min_y = std::min(*min_y, mark_y);
    *max_x = std::max(*max_x, mark_x);
    *max_y = std::max(*max_y, mark_y);
    ROS_INFO("-------------------------");
    ROS_INFO("-------------------------");
    ROS_INFO("*min_x:     %f", *min_x);
    ROS_INFO("*min_y:     %f", *min_y); 
    ROS_INFO("*max_x:     %f", *max_x);
    ROS_INFO("*max_y:     %f", *max_y);
    ROS_INFO("-------------------------");
    ROS_INFO("-------------------------");
    ROS_INFO("-------------------------");
    ROS_INFO("-------------------------");
    
        matrixToMap(kanon);


}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
    // ROS_INFO("Here it should update the Costs");
    if (!enabled_)
    return;



    ROS_INFO("-------------------------");
    ROS_INFO("------------updateCosts--");
    ROS_INFO("min_i:      %i", min_i);
    ROS_INFO("min_j:      %i", min_j); 
    ROS_INFO("max_i:      %i", max_i);
    ROS_INFO("max_j:      %i", max_j);
    ROS_INFO("-------------------------");
    ROS_INFO("-------------------------");
    ROS_INFO("-------------------------");
    ROS_INFO("-------------------------"); 

    min_i = 1000;
    min_j = 1000;
    max_i = 3000;
    max_j = 3000;

    ROS_INFO("-------------------------");
    ROS_INFO("------------updateCosts--");
    ROS_INFO("min_i:      %i", min_i);
    ROS_INFO("min_j:      %i", min_j); 
    ROS_INFO("max_i:      %i", max_i);
    ROS_INFO("max_j:      %i", max_j);
    ROS_INFO("-------------------------");
    ROS_INFO("-------------------------");
    ROS_INFO("-------------------------");
    ROS_INFO("-------------------------");

    int j = min_j;
    int i = min_i;
    for (j; j < max_j; j++)
    {
        for (i; i < max_i; i++)
        {
            int index = getIndex(i, j);
            //if (costmap_[index] == NO_INFORMATION)
            // ROS_INFO("Here I have %i = %i", costmap_[index], NO_INFORMATION);
            //continue;
            //ROS_INFO("Here I would publish the map with %i at [%i, %i]", costmap_[index], i, j);
            master_grid.setCost(i, j, costmap_[index]);
        }
    }
    ROS_INFO("i: %i  j: %i", i, j);
}

} // end namespace
