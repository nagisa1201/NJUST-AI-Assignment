#ifndef NAV2_COSTMAP_2D__OBSTACLE_LAYER_HPP_
#define NAV2_COSTMAP_2D__OBSTACLE_LAYER_HPP_

#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace nav2_costmap_2d
{

/**
 * @class ObstacleLayerUltra
 * @brief Takes in laser and pointcloud data to populate into 2D costmap
 */
class ObstacleLayerUltra : public CostmapLayer
{
public:
    /**
     * @brief A constructor
     */
    ObstacleLayerUltra()
    {
        costmap_ = NULL; // this is the unsigned char* member of parent class Costmap2D.
    }

    /**
     * @brief A destructor
     */
    virtual ~ObstacleLayerUltra();
    void onInitialize() override;
    void reset() override;
    void updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double *min_x, double *min_y, double *max_x, double *max_y) override;
    void updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j) override;
    bool isClearable() override { return true; };

protected:
    void laserScanCallback(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan);
    int combination_method_{1}; // 1: overwrite, 2: add, 3: subtract
    double _obstacle_max_range;
    double _obstacle_min_range;
    double _transform_tolerance;
    bool debug_ = true;
    bool rolling_window_ = false;
    rclcpp::Time last_update_time_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_ = nullptr;
    std::string map_frame_;
    std::string base_frame_;
    std::string laser_frame_;
    tf2_ros::Buffer *tf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan>> laser_scan_sub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_pub_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> map_pub_;
    /// @brief Used to project laser scans into point clouds
    laser_geometry::LaserProjection projector_;
};

} // namespace nav2_costmap_2d

#endif // NAV2_COSTMAP_2D__OBSTACLE_LAYER_HPP_