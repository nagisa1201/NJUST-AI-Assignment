#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <algorithm>
#include <costmap_ultra/obstacle_layer.hpp>
#include <memory>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/pcl_ros/transforms.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vector>
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ObstacleLayerUltra, nav2_costmap_2d::Layer)

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_costmap_2d
{

ObstacleLayerUltra::~ObstacleLayerUltra()
{
}

void ObstacleLayerUltra::onInitialize()
{
    bool track_unknown_space;
    // double transform_tolerance;

    // The topics that we'll subscribe to from the parameter server
    std::string topics_string;
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("combination_method", rclcpp::ParameterValue(1));
    declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));
    declareParameter("map_frame", rclcpp::ParameterValue(std::string("map")));
    auto node = node_.lock();
    if (!node)
    {
        throw std::runtime_error{"Failed to lock node"};
    }

    node->get_parameter(name_ + "." + "combination_method", combination_method_);
    node->get_parameter("transform_tolerance", _transform_tolerance);
    node->get_parameter(name_ + "." + "observation_sources", topics_string);
    node->get_parameter(name_ + "." + "map_frame", map_frame_);
    node->get_parameter(name_ + "." + "enabled", enabled_);
    // now we need to split the topics based on whitespace which we can use a stringstream for
    std::stringstream ss(topics_string);
    std::string topic;
    std::string source;
    // enabled_ = true;
    debug_ = false;
    current_ = true; // 奇妙小参数,一定要为true,否则会导致导航阻塞
    matchSize();
    rolling_window_ = layered_costmap_->isRolling();
    while (ss >> source)
    {
        // get the parameters for the specific topic

        declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
        declareParameter(source + "." + "obstacle_max_range", rclcpp::ParameterValue(2.5));
        declareParameter(source + "." + "obstacle_min_range", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "base_frame", rclcpp::ParameterValue(std::string("base_link")));
        node->get_parameter(name_ + "." + source + "." + "topic", topic);

        // get the obstacle range for the sensor
        node->get_parameter(name_ + "." + source + "." + "obstacle_max_range", _obstacle_max_range);
        node->get_parameter(name_ + "." + source + "." + "obstacle_min_range", _obstacle_min_range);
        node->get_parameter(name_ + "." + source + "." + "base_frame", base_frame_);
    }
    // 创建tf2_ros::Buffer对象
    // tf_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_ = new tf2_ros::Buffer(node->get_clock());
    // 创建tf2_ros::TransformListener对象
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    // 创建订阅者
    laser_scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(topic,
                                                                             rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg)
                                                                             { laserScanCallback(msg); });
    if (debug_)
    {
        std::string node_name = node->get_name();
        pointcloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/debug_pointcloud", rclcpp::SensorDataQoS());
        map_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(node_name + "/debug_map", rclcpp::SensorDataQoS());
    }
}
/**
 * @brief 将雷达数据转化为点云数据，并存储在点云指针中
 *
 * @param scan
 */
void ObstacleLayerUltra::laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan)
{
    laser_frame_ = scan->header.frame_id;
    cloud_ptr_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    float current_angle = scan->angle_min;
    // RCLCPP_INFO(logger_, "Received laser scan with %zu points", scan->ranges.size());
    last_update_time_ = scan->header.stamp;
    auto transform = geometry_msgs::msg::TransformStamped();
    try
    {
        transform = tf_->lookupTransform(
            base_frame_,
            laser_frame_,
            last_update_time_,
            rclcpp::Duration::from_seconds(_transform_tolerance));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(logger_, "Could not transform %s to %s: %s", laser_frame_.c_str(), base_frame_.c_str(), ex.what());
        return; // 如果变换失败，直接返回
    }
    auto x_bias = transform.transform.translation.x;
    auto y_bias = transform.transform.translation.y;
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        float range = scan->ranges[i];

        // 跳过无效点
        if (range < scan->range_min || range > scan->range_max ||
            std::isnan(range) || std::isinf(range))
        {
            current_angle += scan->angle_increment; // 角度递增
            continue;
        }

        // 跳过不在障碍物检测范围内的点
        // if (range < _obstacle_min_range || range > _obstacle_max_range)
        // {
        //     current_angle += scan->angle_increment; // 角度递增
        //     continue;
        // }

        // 计算点坐标并添加到点云
        float x_laser = range * std::cos(current_angle);
        float y_laser = range * std::sin(current_angle);
        float x_base = x_laser + x_bias; // 将点从激光坐标系转换到base_link坐标系
        float y_base = y_laser + y_bias;
        // y_bias += y; // 添加偏移量
        float range_transformed = std::sqrt(x_base * x_base + y_base * y_base);
        if (range_transformed < _obstacle_min_range || range_transformed > _obstacle_max_range)
        {
            current_angle += scan->angle_increment; // 角度递增
            continue;                               // 跳过不在障碍物检测范围内的点
        }
        cloud_ptr_->points.emplace_back(pcl::PointXYZ{x_laser, y_laser, 0.0});

        // 更新角度（注意：应在处理完当前点后更新角度）
        current_angle += scan->angle_increment;
    }

    // 将点云从激光坐标系转换到base_link坐标系
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl_ros::transformPointCloud(*cloud_ptr_, *transformed_cloud, transform);
    // cloud_ptr_->clear();
    // for (auto &point : transformed_cloud->points)
    // {
    //     double range = std::sqrt(point.x * point.x + point.y * point.y);
    //     if (range < _obstacle_min_range || range > _obstacle_max_range)
    //     {
    //         continue; // 跳过不在障碍物检测范围内的点
    //     }
    //     cloud_ptr_->points.emplace_back(pcl::PointXYZ{point.x, point.y, point.z});
    // }
}
void ObstacleLayerUltra::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
{
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    // 要更新原点,定式
    if (rolling_window_)
    {
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }
    useExtraBounds(min_x, min_y, max_x, max_y);
    reset();
    // 获得雷达到map的变换
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_->lookupTransform(
            map_frame_,
            laser_frame_,
            last_update_time_,
            rclcpp::Duration::from_seconds(_transform_tolerance));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(logger_, "Could not transform %s to %s: %s", laser_frame_.c_str(), map_frame_.c_str(), ex.what());
        return;
    }

    if (cloud_ptr_ == nullptr || cloud_ptr_->points.empty())
    {
        return;
    }

    // Convert pcl::PointCloud to sensor_msgs::PointCloud2
    sensor_msgs::msg::PointCloud2 cloud_ros;
    pcl::toROSMsg(*cloud_ptr_, cloud_ros);
    cloud_ros.header.frame_id = laser_frame_;
    cloud_ros.header.stamp = last_update_time_;

    // Transform point cloud
    sensor_msgs::msg::PointCloud2 map_cloud_ros;
    try
    {
        tf2::doTransform(cloud_ros, map_cloud_ros, transform);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(logger_, "TF2 transform failed: %s", ex.what());
        return;
    }
    if (debug_)
    {
        pointcloud_pub_->publish(map_cloud_ros);
    }
    auto map_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(map_cloud_ros, *map_pointcloud_ptr);

    // 将点云转换为代价地图
    for (const auto &point : map_pointcloud_ptr->points)
    {
        if (std::isnan(point.x) || std::isnan(point.y))
        {
            continue; // Skip invalid points
        }
        unsigned int mx, my;

        if (worldToMap(point.x, point.y, mx, my))
        {
            // 设置代价为致命障碍物
            costmap_[getIndex(mx, my)] = LETHAL_OBSTACLE;
            touch(point.x, point.y, min_x, min_y, max_x, max_y);
        }
        else
        {
            RCLCPP_DEBUG(logger_, "Point (%f, %f) is out of bounds, skipping", point.x, point.y);
        }
    }
    // 打印costmap障碍信息
    if (debug_)
    {
        RCLCPP_INFO(logger_, "Updated bounds: min_x=%f, min_y=%f, max_x=%f, max_y=%f,robot_x=%f,robot_y=%f",
                    *min_x, *min_y, *max_x, *max_y, robot_x, robot_y);
    }
}
/**
 * @brief 根据缓冲区的地图更新主地图的代价
 *
 * @param master_grid
 * @param min_i
 * @param min_j
 * @param max_i
 * @param max_j
 */
void ObstacleLayerUltra::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                     int min_i, int min_j, int max_i, int max_j)
{

    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    switch (combination_method_)
    {
    case 0: // Overwrite
        updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
        break;
    case 1: // Maximum
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        break;
    default: // Nothing
        break;
    }
}
/**
 * @brief 清空代价地图，将所有单元格设置为FREE_SPACE
 *
 */
void ObstacleLayerUltra::reset()
{
    // Reset the costmap to free space
    std::fill(costmap_, costmap_ + getSizeInCellsX() * getSizeInCellsY(), FREE_SPACE);
    // RCLCPP_INFO(logger_, "xy: %d, %d", getSizeInCellsX(), getSizeInCellsY());

} // namespace nav2_costmap_2d
} // namespace nav2_costmap_2d