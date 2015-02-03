#include "ros_fuse_point_cloud/pcld_fuser.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_fuser");

  SyncPointCloudStacker<pcl::PointXYZRGB> fuser;

  ros::Rate rate(5);

  ros::AsyncSpinner aspin(1);
  aspin.start();

  while (ros::ok())
  {
    fuser.fuseAndPub();

    rate.sleep();
  }

  aspin.stop();

  return 1;
}
