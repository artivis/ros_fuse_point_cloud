/*
* Software License Agreement (Modified BSD License)
*
* Copyright (c) 2014, PAL Robotics, S.L.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of PAL Robotics, S.L. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
/** \author Jeremie Deray. */

/**
* This sample shows how to inherit from class SyncPointCloudHandler
*
* It subscribes to a given list of point cloud topics, synchronises them
* and save them in a given folder.
*/

#ifndef ROS_FUSE_POINT_CLOUD_PCLD_FUSER_H
#define ROS_FUSE_POINT_CLOUD_PCLD_FUSER_H

// ROS headers
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

// PCL header
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

// BOOST header
#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>

#include <ros_img_sync/sync_pointcloud_handler.h>

template <typename T>
class SyncPointCloudStacker :
  public SyncPointCloudHandler
{

public:

  /*
  * Constructor.
  * Retrieve rosparam 'topics' as a list of image topics to synchronise
  * Listen to several PointCloud, tranform them to a target_frame
  * and save them all in a single pcd file
  */
  SyncPointCloudStacker();

  /*
  * Destructor.
  */
  ~SyncPointCloudStacker() {}

  bool waitForFused(sensor_msgs::PointCloud2& stack,
                    ros::Duration timer = ros::Duration(0));

  bool fuseAndPub(ros::Duration timer = ros::Duration(0));

  bool tfPcldRefFrame(sensor_msgs::PointCloud2& inputCloud,
                      sensor_msgs::PointCloud2& outputCloud,
                      const std::string& frame = "");

  void downSample(pcl::PointCloud<T>& inputCloud,
                  pcl::PointCloud<T>& outputCloud,
                  double leafSize);

private:

  ros::Publisher _fuse_pub;
  tf::TransformListener _sub_tf;

  boost::mutex _mut;

  std::string _refFrame;
  double _leaf_size;
  bool _downsample;
  bool _tf_pcl;
  int _seq;
};

template <typename T>
SyncPointCloudStacker<T>::SyncPointCloudStacker() :
  SyncPointCloudHandler(), // don't forget to call base constructor
  _seq(0),
  _leaf_size(0.05),
  _downsample(false),
  _tf_pcl(false),
  _refFrame("/base_link")
{
  bool isStart = start();

  if (!isStart)
    _nh.shutdown();

  _nh.param("reference_frame", _refFrame, _refFrame);

  _nh.param("leaf_size", _leaf_size, _leaf_size);
  _nh.param("downsample", _downsample, _downsample);

  _nh.param("transform_pointcloud", _tf_pcl, _tf_pcl);

  ROS_INFO("Fused cloud frame id : %s", _refFrame.c_str());

  if (_downsample)
    ROS_INFO("Downsample enabled with leaf size %f", _leaf_size);

  if (_tf_pcl)
    ROS_INFO("Pointcloud Transformation enabled.");

  _fuse_pub = _nh.advertise<sensor_msgs::PointCloud2>("fused_cloud", 1);
}

template <typename T>
bool SyncPointCloudStacker<T>::waitForFused(sensor_msgs::PointCloud2& stack,
                                            ros::Duration timer)
{
  std::vector<sensor_msgs::PointCloud2> pointClouds;

  pcl::PointCloud<T> stack_cloud;

  // Retrieve sensor_msgs
  bool ok = waitForPointClouds(pointClouds, timer);

  if (!ok)
    return false;
  else
  {
    // Convert sensor_msgs to pcl & stack them
    for (size_t i=0; i<pointClouds.size(); ++i)
    {
      pcl::PointCloud<T> cloud;

      if (_tf_pcl)
        if (!tfPcldRefFrame(pointClouds[i], pointClouds[i]))
          return false;

      pcl::fromROSMsg(pointClouds[i], cloud);
      stack_cloud += cloud;
    }

    if (_downsample)
      downSample(stack_cloud, stack_cloud, _leaf_size);

    pcl::toROSMsg(stack_cloud, stack);
  }

  return (stack.width > 0)? true : false;
}

template <typename T>
bool SyncPointCloudStacker<T>::fuseAndPub(ros::Duration timer)
{
  sensor_msgs::PointCloud2 stack;

  bool ok = waitForFused(stack, timer);

  ROS_DEBUG_STREAM("ok ? " << ok);

  if (ok)
  {
    stack.header.frame_id = _refFrame;
    stack.header.stamp = ros::Time::now();
    stack.header.seq = ++_seq;

    _fuse_pub.publish(stack);
  }

  return ok;
}

template <typename T>
bool SyncPointCloudStacker<T>::tfPcldRefFrame(sensor_msgs::PointCloud2& inputCloud,
                                              sensor_msgs::PointCloud2& outputCloud,
                                              const std::string& frame)
{
  std::string inframe = (!strcmp(frame.c_str(), "")) ? _refFrame : frame;

  if (!strcmp(inputCloud.header.frame_id.c_str(),
              inframe.c_str()))
  {
    outputCloud = inputCloud;
    return true;
  }

  boost::mutex::scoped_lock lock(_mut);

  // Transform point cloud to map frame and store as pcl
  tf::StampedTransform transform;
  try
  {
    if(_sub_tf.waitForTransform(inframe, inputCloud.header.frame_id,
                                inputCloud.header.stamp, ros::Duration(1,0)))
      _sub_tf.lookupTransform(inframe, inputCloud.header.frame_id,
                              inputCloud.header.stamp, transform);
    else
    {
      ROS_ERROR("Timeout, couldn't retrieve tf.");
      return false;
    }
  }
  catch (tf::LookupException &e)
  {
    ROS_ERROR ("%s", e.what());
    return false;
  }
  catch (tf::ExtrapolationException &e)
  {
    ROS_ERROR ("%s", e.what());
    return false;
  }

  pcl_ros::transformPointCloud(inframe, transform, inputCloud, outputCloud);

  return true;
}

template <typename T>
void SyncPointCloudStacker<T>::downSample(pcl::PointCloud<T>& inputCloud,
                                          pcl::PointCloud<T>& outputCloud,
                                          double leafSize)
{
  pcl::VoxelGrid<T> vg;

  typename pcl::PointCloud<T>::Ptr pclPtr = inputCloud.makeShared();

  vg.setInputCloud(pclPtr);
  vg.setLeafSize(leafSize, leafSize, leafSize);
  vg.setDownsampleAllData(true);
  vg.filter(outputCloud);
}

#endif //ROS_FUSE_POINT_CLOUD_PCLD_FUSER_H
