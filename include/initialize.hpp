#ifndef Initializer_HPP
#define Initializer_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath> 

// for rotation
#include <pcl/common/transforms.h>

// for cropbox
#include <pcl/filters/crop_box.h>

// for RANSAC
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using std::placeholders::_1;

class Initializer : public rclcpp::Node
{
public:
  Initializer();

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2 &msg) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

#endif // Initializer_HPP