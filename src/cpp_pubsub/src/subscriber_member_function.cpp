// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "open3d_ros.h"
#include "detectblocks.h"
#include "open3d/Open3D.h"


using std::placeholders::_1;

using namespace open3d;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/color/points", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I received the message , height is: '%d'", msg->height); //
      //std::cout << "Complete ROS cloud is: " << msg.get() << std::endl;      
      RCLCPP_INFO(this->get_logger(), "I received the message v1.0!"); //
      //Pass this onto the block detector...

      open3d::geometry::PointCloud source;
      geometry_msgs::msg::TransformStamped xform;
      //orient the point cloud such that the surface is horizontal:
      tf2::Quaternion tf2_quat, tf2_quat_from_msg;
      tf2_quat.setRPY(0.006, 0.960, 0.007);

      //Now rotate around the y:
      tf2::Quaternion tf2_quat_tilt;
      tf2_quat_tilt.setRPY(0, -1.04, 0);

      tf2::Quaternion tf2_quat_total = tf2_quat *  tf2_quat_tilt;
      geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
      
      xform.transform.rotation.x = 0.654;
      xform.transform.rotation.y = -0.653;
      xform.transform.rotation.z = 0.2690;
      xform.transform.rotation.w = -0.272;

      std::cout << xform.transform.rotation.x << std::endl;

      /*
      0.654, -0.653, 0.269, -0.272
      */

      xform.transform.translation = geometry_msgs::msg::Vector3();
      xform.transform.translation.x = 0.0;
      xform.transform.translation.y = 0.0;
      xform.transform.translation.z = -1.37;

      float floor_height = -5.0;
      Eigen::Vector3d closestPointForward, closestPointRight, closestPointLeft;

      open3d_ros::rosToOpen3d(msg, source, xform, floor_height,
        closestPointForward, closestPointRight, closestPointLeft);

      //Save the point cloud for later debugging:
      auto plyfilename ="/home/valerie/sample_ros2/debugdetectblock.ply";
      open3d::io::WritePointCloud(plyfilename, source);

      //open3d_ros::rosToOpen3d(msg, source, false);

      auto coord_axis = geometry::TriangleMesh::CreateCoordinateFrame(1.0, Eigen::Vector3d(0,0,0));

      std::shared_ptr<geometry::PointCloud> source_ptr =
          std::make_shared<geometry::PointCloud>(source);
      visualization::DrawGeometries({source_ptr, coord_axis}, "Current Cloud");
      std::vector<double> horizontal_surface_heights;
      std::vector<DetectedBlock> block_list;

      Open3DPointCloud o3dpc(0.03f, false, DebugLevel::Verbal );
      
      
      o3dpc.SegmentBlocks(
        *source_ptr,
        horizontal_surface_heights,
        block_list);

      std::vector<std::shared_ptr<const geometry::Geometry>> geometry_ptrs;
      geometry_ptrs.push_back(source_ptr);
      std::cout << "----FOUND " << block_list.size() << " BLOCKS!-------" << std::endl;
      for (auto &block : block_list){
        auto box_top_center_marker = geometry::TriangleMesh::CreateSphere(0.01);  //1 cm sphere
        box_top_center_marker->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
        box_top_center_marker->Translate(block.block_top_center);
        geometry_ptrs.push_back(box_top_center_marker);

        Eigen::Vector3d end_point = block.block_top_center + block.block_normal*0.1;
        //pretend like it is perfectly horizontal:
        end_point[2] = block.block_top_center[2];
        auto normal_line = o3dpc.CreateLine(block.block_top_center, end_point);  //1 cm
        normal_line.PaintUniformColor(Eigen::Vector3d(0.10, 0.92, 0.8));
        auto normal_line_ptr = std::make_shared<geometry::LineSet>(normal_line);           
        geometry_ptrs.push_back( normal_line_ptr);
        geometry_ptrs.push_back(coord_axis);

        std::cout << "center: " << block.block_top_center << std::endl;
        std::cout << "normal: " << block.block_normal << std::endl;
        std::cout << "z rotation: " << o3dpc.to_degrees(block.angle_around_z) << std::endl;
        
        std::cout << "---------------------------------" << std::endl;
      }

      visualization::DrawGeometries(geometry_ptrs, 
                                      "Detected Blocks Result");

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
