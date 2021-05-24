/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    @brief ROS node for detecting obstacles in a point cloud.

*/

#include <ros/ros.h>
#include <velo_cam_calibration/velodyne_vertex.h>


/** Main entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne_vertex_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create height map class, which subscribes to velodyne_points

  Velodyne hm_front(node, priv_nh);
  //Velodyne hm_front(node, priv_nh, argc);
  // isr loop
  while (ros::ok()){
      // handle callbacks until shut down
      ros::spinOnce();//ros::spin();
  }

  return 0;
}

