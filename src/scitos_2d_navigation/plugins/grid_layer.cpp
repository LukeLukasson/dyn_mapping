#include <scitos_2d_navigation/grid_layer.h>
#include <pluginlib/class_list_macros.h>

// debug
#include "std_msgs/String.h"
#include <vector>               // needed for updating map
#include <map>                  // needed for converting Eigen matrix to array
#include <Eigen/StdVector>      // needed for converting Eigen matrix to array
#include <sys/types.h>          // for getting int8_t

PLUGINLIB_EXPORT_CLASS(scitos_2d_navigation::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::NO_INFORMATION;

// Luke
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::VectorXi;

/* ToDo
 * ====
 * 
 * o Handle resolution and size of /map generically
 * o Clean up code
 * 
 * Done
 * ====
 * 
 * x Parameters by reference in functions? Why better? -> see schlachtfeld
 * x initStaticMap does not need an argument!!!
 */
 
 
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
    dynamicMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/dynamic_map", 10);
    chatterPub = nh.advertise<std_msgs::String>("/chatter", 1000);
    // define subscriber
    //~ mainMapSub = nh.subscribe("/map", 2, &GridLayer::initStaticMapCallback, this); 
    
    // initialize an nxn matrix with any scalar (ColumnMajor!!!)
    int width_meter = 60;
    int height_meter = 30;
    resolution = 0.05;
    width = width_meter / resolution;
    height = height_meter / resolution;

    n_cells = width*height;
    
    staticMap_matrix = MatrixXf::Constant(width,height,0.5);

    //~ std::cout << std::endl << staticMap_matrix << std::endl;

    publishMaps();
    
    //initStaticMap(staticMap);
    
    // flags
    flag_before_init = false;
    
}

// initialize static map
void GridLayer::initStaticMap()
{
    ROS_WARN("+++ Initializing static map");
    
    // handeling nav_msgs/MapMetaData
    staticMap.info.resolution = resolution;                         // float32
    staticMap.info.width = width;                                   // uint32
    staticMap.info.height = height;                                 // uint32
    staticMap.info.origin.position.x = -width/2 * resolution;       // same origin as /map
    staticMap.info.origin.position.y = -height/2 * resolution;      // same origin as /map
    staticMap.info.origin.orientation.w = 1.0;                      // same orientation as /map
    int p[n_cells];
    std::vector<signed char> a(p, p+n_cells);
    staticMap.data = a;
}

// initialize dynamic map
void GridLayer::initDynamicMap()
{
    ROS_WARN("+++ Initializing dynamic map");
    
    // handeling nav_msgs/MapMetaData
    dynamicMap.info.resolution = resolution;                         // float32
    dynamicMap.info.width = width;                                   // uint32
    dynamicMap.info.height = height;                                 // uint32
    dynamicMap.info.origin.position.x = -width/2 * resolution;       // same origin as /map
    dynamicMap.info.origin.position.y = -height/2 * resolution;      // same origin as /map
    dynamicMap.info.origin.orientation.w = 1.0;                      // same orientation as /map
    int p[n_cells];
    std::vector<signed char> a(p, p+n_cells);
    dynamicMap.data = a;
    
    // itialize matrix with 0.5 probability (unknown)
    dynamicMap_matrix = MatrixXf::Constant(width,height,0.5);

}

// push values of matrix to OccupancyGrid of map
void GridLayer::publishMaps()
{
    // how many cells in map? -> n_cells
    // cast <float> matrix to <int> matrix
    MatrixXi matrix_int_stat = staticMap_matrix.cast<int>();
    MatrixXi matrix_int_dyn = dynamicMap_matrix.cast<int>();
    //~ std::cout << matrix_int << std::endl;
    
    // transform Eigen::Matrix to Eigen::Vector
    VectorXi vector_int_stat = VectorXi::Map(matrix_int_stat.data(), n_cells);
    VectorXi vector_int_dyn = VectorXi::Map(matrix_int_dyn.data(), n_cells);
        
    // create vector to publish map
    int init_v_stat[n_cells];
    std::vector<signed char> map_vector_stat(init_v_stat, init_v_stat+n_cells);
    int init_v_dyn[n_cells];
    std::vector<signed char> map_vector_dyn(init_v_dyn, init_v_dyn+n_cells);    
    
    // convert vector of <int> to <int8_t> (signed char)
    for( int i=0; i<n_cells; i++ ) {
        map_vector_stat[i] = 100*(int8_t)vector_int_stat[i];
        map_vector_dyn[i] = 100*(int8_t)vector_int_dyn[i];
    }
    
    // publish map
    ROS_WARN("+++ Publishing maps!");
    staticMap.data = map_vector_stat;
    staticMapPub.publish(staticMap);
    dynamicMap.data = map_vector_dyn;
    dynamicMapPub.publish(dynamicMap);
    
    
    //~ // debug
    //~ std_msgs::String msg;
    //~ std::stringstream ss;
    //~ ss << "hello world, msg";
    //~ msg.data = ss.str();
    //~ chatterPub.publish(msg);
    //~ 
    //~ int a[n_cells];   
    //~ int *p = &a[0];    
    //~ Eigen::Map<Eigen::Matrix<int,2,2,Eigen::RowMajor> >(p,2,2) = matrix_int;
    //~ 
    //~ for (int i=0; i<n_cells; i++)
        //~ std::cout << p[i] << " ";
    //~ std::cout << std::endl;


    // transform Eigen::Vector to std::vector
    //~ std::vector<Eigen::VectorXi, Eigen::aligned_allocator<Eigen::VectorXi> > v = vector_int;
    
    //~ v = vector_int.data();
        
    //~ std::cout << v << std::endl;

    //~ std::map<MatrixXi>(v.data(), 2, 2) = matrix_int;

    //~ std::vector<int> v_int;
    //~ std::vector<float> v_float(v_int.begin(), v_int.end());
    
    //~ for( std::vector<signed char>::const_iterator i = v.begin(); i != v.end(); ++i) {
        //~ std::cout << *i << ' ';
    //~ }
}

// init static map by waiting for the original map
void initStaticMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &mainMap)
{
    ROS_WARN("I got the map!!!");    
}

// transform world coordinates to the matrix
void GridLayer::transformMapToMatrix(int map_x, int map_y, int &matrix_x, int &matrix_y)
{
    //~ ROS_WARN("+++ Let's transform!");
    
    //~ std::cout << "map_x: " << map_x << " -- map_y: " << map_y << std::endl;
    
    int map_x_max = 4000;
    int map_y_max = 4000;
    
    if (resolution == 0.05) {
        matrix_x = map_x - (map_x_max/2 - width/2);
        matrix_y = map_y - (map_y_max/2 - height/2);
        //~ std::cout << "mat_x: " << matrix_x << " -- mat_y: " << matrix_y << std::endl;

    } else {
        ROS_ERROR("This resolution is not supported yet");
    }
}



void GridLayer::matchSize()
{ 
    Costmap2D* master = layered_costmap_->getCostmap();
    // resolution_ = layered_costmap->getResolution(); // Luke: Maybe usefull for a more generic approach... (as seen in inflation_layer)
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
        setCost(mx+1, my+1, LETHAL_OBSTACLE);
        
        // similiar to worldToMap/setCost - Combo
        //~ int matrix_x;          // Luke: matrix coordinates
        //~ int matrix_y;
        //~ transformMapToMatrix(mx, my, matrix_x, matrix_y);
        //~ std::cout << "mat_x: " << matrix_x << " -- mat_y: " << matrix_y << std::endl;
        //~ staticMap_matrix(matrix_x, matrix_y) = 100;

    }
    
    unsigned char old_cost = getCost(mx, my);
    ROS_WARN_STREAM("Old cost of (" << mx << ", " << my << ") is " << (int)old_cost);
    //~ setCost(mx, my, LETHAL_OBSTACLE);

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

    // update bounds -> only add but never substract from previous maps
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
    
    //~ publishMaps();


}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
    // ROS_INFO("Here it should update the Costs");
    if (!enabled_)
    return;


//~ 
    //~ ROS_INFO("-------------------------");
    //~ ROS_INFO("------------updateCosts--");
    //~ ROS_INFO("min_i:      %i", min_i);
    //~ ROS_INFO("min_j:      %i", min_j); 
    //~ ROS_INFO("max_i:      %i", max_i);
    //~ ROS_INFO("max_j:      %i", max_j);
    //~ ROS_INFO("-------------------------");
    //~ ROS_INFO("-------------------------");
    //~ ROS_INFO("-------------------------");
    //~ ROS_INFO("-------------------------"); 

    //~ min_i = std::max(min_i, 4000/2 - width/2);
    //~ min_j = std::max(min_j, 4000/2 - height/2);
    //~ max_i = std::min(max_i, 4000/2 + width/2);
    //~ max_j = std::min(max_j, 4000/2 + height/2);

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

    costmap_2d::Costmap2D* layered_costmap = layered_costmap_->getCostmap();
    //~ unsigned char* map_master = layered_costmap->getCharMap();
    unsigned int size_x = layered_costmap->getSizeInCellsX(), size_y = layered_costmap->getSizeInCellsY();
    unsigned char* master_array = layered_costmap->getCharMap();
    
    //~ unsigned char* master_array = master_grid.getCharMap();
    //~ unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
    unsigned int array_index = 2196;
    unsigned char element = master_array[array_index];

    std::cout << master_grid.getCharMap() << std::endl;
    std::cout << master_array << std::endl;
    int counter_ma = 0;
    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            int index_ma = layered_costmap->getIndex(i, j);
            //~ std::cout << (int)master_array[index_ma] << " ";
            counter_ma++;
        }
    }
    std::cout << counter_ma << std::endl;
    
    ROS_WARN_STREAM("getCharMap(): " << master_grid.getCharMap() << " master_array: " << master_array << " master_array[0]: " << element);



    int matrix_x, matrix_y;
    int counter = 0;
    for (int j = min_j+1; j < max_j-1; j++)
    {
        for (int i = min_i+1; i < max_i-1; i++)
        {
            counter++;
            int index = layered_costmap->getIndex(i, j); // old: getIndex(i, j);
            //~ master_grid.setCost(i, j, LETHAL_OBSTACLE);
            //~ ROS_INFO_STREAM("Here I have a costmap with " << (int)costmap_[index]);
            if ((int)master_array[index] == NO_INFORMATION) { // old: costmap_[index]
                ROS_INFO("Here I have %i = %i", costmap_[index], NO_INFORMATION);
                continue;
            }
            master_grid.setCost(i, j, (int)master_array[index]); // costmap_[index]
            transformMapToMatrix(i, j, matrix_x, matrix_y);
            //~ std::cout << "mat_x: " << matrix_x << " -- mat_y: " << matrix_y << std::endl;
            //~ staticMap_matrix(matrix_x, matrix_y) = std::max((int)staticMap_matrix(matrix_x, matrix_y), (int)master_array[index]);
            //~ ROS_INFO("i: %i  j: %i", i, j);
        }
    }
    
    if(flag_before_init) {
        // mark end points
        transformMapToMatrix(max_i, min_j, matrix_x, matrix_y);
        staticMap_matrix(matrix_x, matrix_y) = 150;
        transformMapToMatrix(min_i, max_j, matrix_x, matrix_y);
        staticMap_matrix(matrix_x, matrix_y) = 150;
        transformMapToMatrix(max_i, max_j, matrix_x, matrix_y);
        staticMap_matrix(matrix_x, matrix_y) = 150;
        transformMapToMatrix(min_i, min_j, matrix_x, matrix_y);
        staticMap_matrix(matrix_x, matrix_y) = 150;
        //~ transformMapToMatrix(min_i+1, min_j, matrix_x, matrix_y);
        //~ staticMap_matrix(matrix_x, matrix_y) = -1;
        //~ transformMapToMatrix(min_i+2, min_j, matrix_x, matrix_y);
        //~ staticMap_matrix(matrix_x, matrix_y) = 50;
        //~ transformMapToMatrix(min_i+3, min_j, matrix_x, matrix_y);
        //~ staticMap_matrix(matrix_x, matrix_y) = 100;
    } else {
        // initialize staticMap with the values of the static_layer
        initStaticMap(); // create the map so it's available
        for(int i=max_i/2-width/2; i<max_i/2+width/2; i++) {
            for(int j=max_j/2-height/2; j<max_i/2+height/2; j++) {
                int index = layered_costmap->getIndex(i, j);
                transformMapToMatrix(i, j, matrix_x, matrix_y);
                if((int)master_array[index] == NO_INFORMATION) {
                    staticMap_matrix(matrix_x, matrix_y) = 0.5;
                    continue;
                } else if((int)master_array[index] == FREE_SPACE) {
                    staticMap_matrix(matrix_x, matrix_y) = 0.5; // hack! problem listed in notes...
                    continue;
                } else if((int)master_array[index] == LETHAL_OBSTACLE) {
                    staticMap_matrix(matrix_x, matrix_y) = 1;
                    continue;
                } else {
                    ROS_WARN("Not a known value...");
                }
            }
        }
        
        // initialize dynamicMap
        initDynamicMap();
        // only do that once
        flag_before_init = true;
    }
    

    
    layered_costmap->setCost(3, 3, 254); // does not do anything... ???
    std::cout << "Counter: " << counter << " NO_INFO means: " << (int)NO_INFORMATION << std::endl;
    
    publishMaps();

}

} // end namespace
