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
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <chrono>
using std::placeholders::_1;
using namespace std::literals::chrono_literals;
using namespace open3d;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      this->declare_parameter("use_transform_system", false);  //tested
      this->declare_parameter("source_frame", "camera_color_optical_frame"); 
      this->declare_parameter("target_frame", "link_chassis");
      this->declare_parameter("debug_level", "Visual");
      this->declare_parameter("pointcloud_topic", "/camera/depth/color/points"); //tested
      this->declare_parameter("save_ply", false);  //tested
      this->declare_parameter("ply_filename", "./plys/debugdetectblock2.ply");
      
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description = "How high, in meters are you holding the camera off the floor.";

      this->declare_parameter("manual_camera_height", -1.37 ,  param_desc);  //tested
      param_desc.description = "Tilt in degrees.  Forward will be a - number. for example, -45.0 if holding the camera at a 45 degree forward angle.";
      this->declare_parameter("manual_camera_tilt", -45.0, param_desc );  //tested

      std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
      RCLCPP_INFO(this->get_logger(), "Pointcloud Topic:  " + pointcloud_topic);

      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    bool use_transform_system = this->get_parameter("use_transform_system").as_bool();
    std::string source_frame = this->get_parameter("source_frame").as_string();
    std::string target_frame = this->get_parameter("target_frame").as_string();
    bool save_ply = this->get_parameter("save_ply").as_bool();

    
    RCLCPP_INFO(this->get_logger(), "I received the message v1.1!"); //
    //Pass this onto the block detector...
    //look up the transform:
    geometry_msgs::msg::TransformStamped xform;
    open3d::geometry::PointCloud source;

    //bool use_transform_system = false;
    if (use_transform_system){
    try{ 
      xform = tf_buffer_->lookupTransform(target_frame, source_frame,
        msg->header.stamp, 2000ms); 
    }
    catch (std::exception &ex) {
      std::cout << "Tranform Error:  " << ex.what()
                  << std::endl;
      return;
    }        
    }
    else {

      double CAMERA_HEIGHT =  this->get_parameter("manual_camera_height").as_double();
      double CAMERA_TILT =  this->get_parameter("manual_camera_tilt").as_double();


      tf2::Quaternion tf2_quat, tf2_quat_from_msg;

      //FROM CAMERA DEPTH FRAME TO Z up, X forward Frame
      tf2_quat.setRPY( TO_RADIANS * -90.0, 0.0, -90.0 * TO_RADIANS);

      tf2::Quaternion tf2_quat_tilt;
      tf2_quat_tilt.setRPY(TO_RADIANS * CAMERA_TILT, 0.0, 0.0);  //45 degree tilt
      tf2::Quaternion tf2_quat_total =  tf2_quat * tf2_quat_tilt;
      geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat_total);
      xform.transform.rotation = msg_quat;
      xform.transform.translation = geometry_msgs::msg::Vector3();
      xform.transform.translation.x = 0.0;
      xform.transform.translation.y = 0.0;
      //Camera Height:
      xform.transform.translation.z = -1.0 * CAMERA_HEIGHT;
    }

    float floor_height = -1 * xform.transform.translation.z + 0.10;
    Eigen::Vector3d closestPointForward, closestPointRight, closestPointLeft;

    open3d_ros::rosToOpen3d(msg, source, xform, floor_height,
      closestPointForward, closestPointRight, closestPointLeft);

    

    //Keep an unaderated copy of the original for saving later.
    geometry::PointCloud source_orig = source;
    
    std::shared_ptr<geometry::PointCloud> source_ptr = source.UniformDownSample(2);
    auto coord_axis = geometry::TriangleMesh::CreateCoordinateFrame(1.0, Eigen::Vector3d(0,0,0));

    visualization::DrawGeometries({source_ptr, coord_axis}, "Current Cloud");
    std::vector<double> horizontal_surface_heights;
    std::vector<DetectedBlock> block_list;

    DebugLevel debug_level = DebugLevel::None;
    std::string debug_level_param = this->get_parameter("debug_level").as_string();

    if (debug_level_param == "Verbal"){
      debug_level = DebugLevel::Verbal;
    }
    else if (debug_level_param == "Visual"){
      debug_level = DebugLevel::Visual;
    }

    Open3DPointCloud o3dpc(0.03f, false, debug_level );
    
    auto start_time = std::chrono::high_resolution_clock::now();
    o3dpc.SegmentBlocks(
      *source_ptr,
      horizontal_surface_heights,
      block_list, 
      -1.3);
    auto end_time = std::chrono::high_resolution_clock::now();
    if (debug_level == DebugLevel::None){
      std::cout << "Time difference:"
      << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms" << std::endl;
    }

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

    //Save the point cloud for later debugging:
    std::cout << "save_ply?" << save_ply << std::endl;
    if (save_ply){
      std::string plyfilename = this->get_parameter("ply_filename").as_string();

      std::cout << "Saving ply to" << plyfilename << std::endl;


      try {
        open3d::io::WritePointCloud(plyfilename, source_orig);
      }
      catch (std::exception &ex) {
        std::cout << "Error Saving Ply:  " << ex.what()
                  << std::endl;
      }        
    }

  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  const double TO_RADIANS = 0.0174533;  

};

double toRadians(double degrees){
  return 0.0174533 * degrees;

}



void quaternion_test(){

  tf2::Quaternion tf2_quat_tilt, tf2_quat;
  tf2_quat.setRPY(toRadians(-120.0), 0.0, toRadians(-90.0));  //45 degree tilt

  tf2_quat_tilt.setRPY(  toRadians(-30.0), toRadians(0.0),  toRadians(0.0));  //45 degree tilt
  std::cout << "single xform: " << tf2_quat.x() << std::endl;
  std::cout << "  : " << tf2_quat.y() << std::endl;
  std::cout << "  : " << tf2_quat.z() << std::endl;
  std::cout << "  : " << tf2_quat.w() << std::endl;

  tf2_quat.setRPY(toRadians(-90.0), 0.0, toRadians(-90.0));  //45 degree tilt
  std::cout << "-----------------_ " << std::endl;

  tf2::Quaternion tf2_quat_total =  tf2_quat * tf2_quat_tilt;

  std::cout << "total xform: " << tf2_quat_total.x() << std::endl;
  std::cout << "  : " << tf2_quat_total.y() << std::endl;
  std::cout << "  : " << tf2_quat_total.z() << std::endl;
  std::cout << "  : " << tf2_quat_total.w() << std::endl;


  tf2_quat_total =  tf2_quat_tilt * tf2_quat;

  std::cout << "total xform: " << tf2_quat_total.x() << std::endl;
  std::cout << "  : " << tf2_quat_total.y() << std::endl;
  std::cout << "  : " << tf2_quat_total.z() << std::endl;
  std::cout << "  : " << tf2_quat_total.w() << std::endl;

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
