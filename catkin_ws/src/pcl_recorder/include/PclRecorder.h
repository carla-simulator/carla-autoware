/*
 * Copyright (c) 2019 Intel Labs.
 *
 * authors: Frederik Pasch (frederik.pasch@intel.com)
 */
#pragma once

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

class PclRecorder
{
public:

  PclRecorder();

  void callback(const pcl::PCLPointCloud2::ConstPtr& cloud);

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener *tfListener;
  static constexpr const char* fixed_frame_ = "map";

};
