/* -*- mode: C++ -*- */
/*  Copyright (C) 2010 UT-Austin & Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 * 
 *  License: Modified BSD Software License 
 */

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/distances.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/intersections.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>

//ROS include
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <velodyne_pointcloud/point_types.h>
// random number
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <ctime>
#include "tinyxml.h"


#include <Eigen/Dense>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include <opencv2/core/core.hpp>

// shorter names for point cloud types in this namespace
typedef pcl::PointXYZRGB RGBPoint;
typedef pcl::PointCloud<RGBPoint> RGBPointCloud;
typedef pcl::PointXYZ PPoint;
typedef pcl::PointCloud<PPoint> PPointCloud;
typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointXY VPoint2D;
typedef pcl::PointCloud<VPoint2D> VPoint2DCloud;
void randomize(void);

#define PIXEL_W 1280
#define PIXEL_H 720
#define SCAN_LINE 32

static int _scan_line;
static bool _click_filter_pts;

static double _filter_xmin;
static double _filter_xmax;
static double _filter_ymin;
static double _filter_ymax;
static double _filter_zmin;
static double _filter_zmax;

static double _distance_thresh;
static double _left_line_real_length; // m
static double _right_line_real_length;
static double _error_thresh;


class Velodyne
{
public:

  /** Constructor
   *
   *  @param node NodeHandle of this instance
   *  @param private_nh private NodeHandle of this instance
   */
  Velodyne(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Velodyne();

  /** callback to process data input
   *
   *  @param scan vector of input 3D data points
   *  @param stamp time stamp of data
   *  @param frame_id data frame of reference
   */
  void RvizClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point);
  void initializeCloud(const VPointCloud::ConstPtr &scan);
  void processData(const VPointCloud::ConstPtr &scan);
  void lineFitting(const PPointCloud::Ptr &line_cloud, Eigen::VectorXf &coefficients_vec);
  void planeFitting(const VPointCloud::ConstPtr &scan, float &A, float &B, float &C, float &D);
  void writeData(Eigen::Vector4f* V);
  void projectOnPlane(const VPointCloud::ConstPtr &scan, float &A, float &B, float &C, float &D);

//  cv::Mat obs_map;

private:


  // Parameters that define the grids and the height threshold
  // Can be set via the parameter server

  bool write = false;
  
  int start_ring;
  int end_ring;

  size_t filtered_count;
  std::vector<size_t> num_pts_on_ring; 
  unsigned int num_of_rings;

  // rviz clicked velodyne points
  std::vector<cv::Point3f> clicked_velodyne_points_;
  float filter_xmin = 100000.;
  float filter_xmax = -100000.;
  float filter_ymin = 100000.;
  float filter_ymax = -100000.;
  float filter_zmin = 100000.;
  float filter_zmax = -100000.;

  // Point clouds generated in processData            
  VPointCloud projected_plane_cloud; // projected point cloud on plane
  std::vector<VPointCloud> ring_projected_cloud;
  PPointCloud UL_line, UR_line, DL_line, DR_line; // points on each line
  RGBPointCloud vertex_cloud; // VU, VL, VD, VR

  // ROS topics
  ros::Subscriber velodyne_scan;
  ros::Publisher xfiltered_publisher;
  ros::Publisher xyfiltered_publisher;
  ros::Publisher xyzfiltered_publisher;
  
  ros::Publisher projected_publisher;
  ros::Publisher vertex_publisher;
  ros::Publisher lines_publisher;
  
  ros::Subscriber     subscriber_clicked_point_;
};
