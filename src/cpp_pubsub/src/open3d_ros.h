#ifndef OPEN3D_ROS_HPP_
#define OPEN3D_ROS_HPP_

// Open3D
#include <open3d/Open3D.h>

// ROS2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
// Eigen
#include <Eigen/Dense>

// C++
#include <string>


#include <tf2_ros/transform_listener.h>
//#include <geometry_msgs/TransformStamped.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <tf2/convert.h>
#include <Eigen/Geometry>
//#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/msg/vector3.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace open3d_ros
{
/**
 * @brief Copy data from a open3d::geometry::PointCloud to a sensor_msgs::PointCloud2
 * 
 * @param pointcloud Reference to the open3d PointCloud
 * @param ros_pc2 Reference to the sensor_msgs PointCloud2
 * @param frame_id The string to be placed in the frame_id of the PointCloud2
 */
void open3dToRos(const open3d::geometry::PointCloud& pointcloud, sensor_msgs::msg::PointCloud2& ros_pc2,
                std::string frame_id = "open3d_pointcloud");

/**
 * @brief Copy data from a sensor_msgs::PointCloud2 to a open3d::geometry::PointCloud
 * 
 * @param ros_pc2 Reference to the sensor_msgs PointCloud2
 * @param o3d_pc Reference to the open3d PointCloud
 * @param skip_colors If true, only xyz fields will be copied
 */

void rosToOpen3d(const  sensor_msgs::msg::PointCloud2::SharedPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc,
                const geometry_msgs::msg::TransformStamped& xform , float floor_height, Eigen::Vector3d& closestPointForward, 
                Eigen::Vector3d& closestPointRight, Eigen::Vector3d& closestPointLeft);

void rosToOpen3d(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc,
                bool skip_colors = false);


}    // namespace open3d_ros

#endif /*OPEN3D_ROS_HPP_*/
