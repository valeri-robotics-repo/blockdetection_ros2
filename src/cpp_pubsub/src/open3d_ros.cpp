#include "open3d_ros.h"

namespace open3d_ros
{
void open3dToRos(const open3d::geometry::PointCloud& pointcloud, sensor_msgs::msg::PointCloud2 ros_pc2, std::string frame_id)
{
    
    //ros_pc2.header.frame_id = frame_id;
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");



    if (pointcloud.HasColors())
    {
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
        for (size_t i = 0; i < pointcloud.points_.size();
             i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
        {
            const Eigen::Vector3d& point = pointcloud.points_[i];
            const Eigen::Vector3d& color = pointcloud.colors_[i];
            *ros_pc2_x = point(0);
            *ros_pc2_y = point(1);
            *ros_pc2_z = point(2);
            *ros_pc2_r = (int)(255 * color(0));
            *ros_pc2_g = (int)(255 * color(1));
            *ros_pc2_b = (int)(255 * color(2));
        }
    }
    else
    {
        for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
        {
            const Eigen::Vector3d& point = pointcloud.points_[i];
            *ros_pc2_x = point(0);
            *ros_pc2_y = point(1);
            *ros_pc2_z = point(2);
        }
    }
}

void rosToOpen3d(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_pc2, 
    open3d::geometry::PointCloud& o3d_pc, 
    const geometry_msgs::msg::TransformStamped& xform, 
    float floor_height,
    Eigen::Vector3d& closestPointForward, 
    Eigen::Vector3d& closestPointRight, 
    Eigen::Vector3d& closestPointLeft)
{
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");


    bool hasColors = true;
    //std::cout << "Point cloud has color data! " << hasColors << std::endl;


    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");

    double closest_fwd_x = 10000;
    double closest_right_norm = 10000;
    double closest_left_norm = 10000;
    uint below_floor_ctr = 0;

    o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
    auto v_fwd_closest = Eigen::Vector3d(0,0,0);
    auto v_right_closest = Eigen::Vector3d(0,0,0);
    auto v_left_closest = Eigen::Vector3d(0,0,0);

    float forward_padding = 0.25;

    geometry_msgs::msg::Quaternion q_msg = xform.transform.rotation;

// Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
//tf2::convert(msg_quat, tf2_quat_from_msg);
// or

    tf2::Quaternion tf2_quat_from_msg;
    tf2::fromMsg(q_msg, tf2_quat_from_msg);
    Eigen::Quaterniond q1(tf2_quat_from_msg.getW(),
    tf2_quat_from_msg.getX(),
    tf2_quat_from_msg.getY(),
    tf2_quat_from_msg.getZ());

    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
	    
        auto cv = Eigen::Vector3d(0,0,0);
        if (hasColors){
            ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b;
            
            cv = Eigen::Vector3d( ((int)(*ros_pc2_r)) / 255.0, ((int)(*ros_pc2_g)) / 255.0,
                                                         ((int)(*ros_pc2_b)) / 255.0 );
        }
        
        auto point_in = Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z);
        Eigen::Vector3d point_out = q1 * point_in;

	    //auto v = Eigen::Vector3d(point_out.point.x, point_out.point.y, point_out.point.z);
        if (point_out[2] > floor_height){
            o3d_pc.points_.push_back(point_out);
            o3d_pc.colors_.push_back(cv);
	    }
        else {
            below_floor_ctr += 1;
            //std::cout << "Below floor, skipping!" << floor_height << " " << point_out[2] <<std::endl;
        }

            
        //point must be above the floor:
        if (point_out[2] > floor_height && point_out[0] > 0.10){ 
            bool isfwdpath = (point_out[1] >= -1.0 * forward_padding && point_out[1] <= forward_padding);

            if (isfwdpath){
                if (point_out[0] < closest_fwd_x){
                    if (abs(point_out[0] - closest_fwd_x) < 0.05){  //If within 5 cm...
                        //choose closest by distance:
                        if (point_out.norm() < v_fwd_closest.norm()){
                            closest_fwd_x = point_out[0];
                        v_fwd_closest = point_out;      
                        }
                    }
                    else {
                        closest_fwd_x = point_out[0];
                        v_fwd_closest = point_out;
                    }
                }
            }
            else {
                bool isrightpath = (point_out[1] < -1.0 * forward_padding);
                //bool isleftpath = (point_out[1] > forward_padding);
                auto dist = sqrt(point_out[0]*point_out[0] + point_out[1]* point_out[1]);
                if (isrightpath && dist < closest_right_norm){
                    closest_right_norm = dist;
                    v_right_closest = point_out;
                }
                else{
                    closest_left_norm = dist;
                    v_left_closest = point_out;
                }
            }
        }
    }
    closestPointForward = v_fwd_closest;
    closestPointRight = v_right_closest;
    closestPointLeft = v_left_closest;

    std::cout << "SKIPPED POINTS BELOW FLOOR:  " << below_floor_ctr << " " << floor_height << std::endl;
}


void rosToOpen3d(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc, bool skip_colors)
{
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");

    double closest_norm = 10000;
    o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
    auto v_closest = Eigen::Vector3d(0,0,0);
    if (ros_pc2->fields.size() == 3 || skip_colors == true)
    {
        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
        {
            auto v = Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z);
            auto dist=v.norm();
            if (dist > 0) {
              o3d_pc.points_.push_back(v);
              
              if (dist < closest_norm){
                 closest_norm = dist;
                 v_closest = v;
              }
            }
        }
        std::cout << "CLOSEST POINT IS: " << v_closest << " at " << closest_norm << std::endl;
    }
    else
    {
        o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);
        if (ros_pc2->fields[3].name == "rgb")
        {
            sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");

            for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
                 ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
            {
                o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
                o3d_pc.colors_.push_back(Eigen::Vector3d(((int)(*ros_pc2_r)) / 255.0, ((int)(*ros_pc2_g)) / 255.0,
                                                         ((int)(*ros_pc2_b)) / 255.0));
            }
        }
        else if (ros_pc2->fields[3].name == "intensity")
        {
            sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_i(*ros_pc2, "intensity");
            for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
                 ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
            {
                o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
                o3d_pc.colors_.push_back(Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
            }
        }
    }
}


}    // namespace open3d_ros
