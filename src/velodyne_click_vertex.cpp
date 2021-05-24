#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/distances.h>

#include <time.h>
#include <math.h>

#include <ctime>
#include "tinyxml.h"

#include <ros/package.h>
#include <Eigen/Dense>
#include <vector>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"


typedef pcl::PointXYZ PPoint;
typedef pcl::PointCloud<PPoint> PPointCloud;

std::vector<cv::Point3f> clicked_velodyne_points_;
PPointCloud vertex_cloud;

float filter_xmin = 100000.;
float filter_xmax = -100000.;
float filter_ymin = 100000.;
float filter_ymax = -100000.;
float filter_zmin = 100000.;
float filter_zmax = -100000.;

float left_line_real_length = 0.3; // 0.905
float right_line_real_length = 0.3;
float error_thresh = 0.07;

void writeData()
{
  ROS_INFO("Write xml file");
  // write .xml
  // Get time
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,80,"%Y-%m-%d-%H-%M-%S", timeinfo);
  std::string str(buffer);

  ROS_INFO("VU = (%.4f, %.4f, %.4f)",clicked_velodyne_points_[0].x,clicked_velodyne_points_[0].y,clicked_velodyne_points_[0].z);
  ROS_INFO("VL = (%.4f, %.4f, %.4f)",clicked_velodyne_points_[1].x,clicked_velodyne_points_[1].y,clicked_velodyne_points_[1].z);
  ROS_INFO("VD = (%.4f, %.4f, %.4f)",clicked_velodyne_points_[2].x,clicked_velodyne_points_[2].y,clicked_velodyne_points_[2].z);
  ROS_INFO("VR = (%.4f, %.4f, %.4f)",clicked_velodyne_points_[3].x,clicked_velodyne_points_[3].y,clicked_velodyne_points_[3].z);

  std::string path = ros::package::getPath("velo_cam_calibration");
  std::string backuppath = path + "/data/vertices_"+ str +".xml";
  path = path + "/data/vertices.xml";

  std::cout << std::endl << "Creating .xml file with vertices in: "<< std::endl << path.c_str() << std::endl;
   
  // Create .xml file with vertices
  TiXmlDocument doc;
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "utf-8", "");
  doc.LinkEndChild( decl );
  TiXmlElement * root = new TiXmlElement( "Vertices" );
  doc.LinkEndChild( root );

  std::string name[4] = {"VU", "VL", "VD", "VR"};
  for (int i = 0;i<4;i++)
  {
    std::ostringstream sstreamx, sstreamy, sstreamz;
    sstreamx << clicked_velodyne_points_[i].x;
    sstreamy << clicked_velodyne_points_[i].y; 
    sstreamz << clicked_velodyne_points_[i].z;
    std::string Vx = sstreamx.str();
    std::string Vy = sstreamy.str();
    std::string Vz = sstreamz.str();

    TiXmlElement * velodyne1_node = new TiXmlElement( name[i] );
    root->LinkEndChild( velodyne1_node );
    velodyne1_node->SetAttribute("x",Vx);
    velodyne1_node->SetAttribute("y",Vy);
    velodyne1_node->SetAttribute("z",Vz);
  }
    
  // Save XML file and copy
  doc.SaveFile(path);
  doc.SaveFile(backuppath);
}


void RvizClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point)
{
	clicked_velodyne_points_.push_back(cv::Point3f(in_clicked_point.point.x,
	                                               in_clicked_point.point.y,
	                                               in_clicked_point.point.z));
	
	std::cout << cv::Point3f(in_clicked_point.point.x,
	                         in_clicked_point.point.y,
	                         in_clicked_point.point.z) << std::endl;
  
  vertex_cloud.header.frame_id = in_clicked_point.header.frame_id;
  // PPoint tmp;
  // tmp.x = in_clicked_point.point.x, tmp.y = in_clicked_point.point.y, tmp.z = in_clicked_point.point.z;
  // vertex_cloud.push_back(tmp);
	std::cout << "Number of points: " << clicked_velodyne_points_.size() << std::endl  << std::endl;
  
  filter_xmin = MIN(in_clicked_point.point.x, filter_xmin);
  filter_xmax = MAX(in_clicked_point.point.x, filter_xmax);
  filter_ymin = MIN(in_clicked_point.point.y, filter_ymin);
  filter_ymax = MAX(in_clicked_point.point.y, filter_ymax);
  filter_zmin = MIN(in_clicked_point.point.z, filter_zmin);
  filter_zmax = MAX(in_clicked_point.point.z, filter_zmax);
	
  if (clicked_velodyne_points_.size() == 4)
	{
    for (int i = 0;i<4;i++)
    {
      vertex_cloud.points[i] = PPoint(clicked_velodyne_points_[i].x, clicked_velodyne_points_[i].y, clicked_velodyne_points_[i].z);
    }
      

    // calculate error
    float error[4];
    error[0] = fabs(left_line_real_length - pcl::euclideanDistance(vertex_cloud.points[0], vertex_cloud.points[1]))/left_line_real_length;
    error[1] = fabs(left_line_real_length - pcl::euclideanDistance(vertex_cloud.points[1], vertex_cloud.points[2]))/left_line_real_length;
    error[2] = fabs(right_line_real_length - pcl::euclideanDistance(vertex_cloud.points[2], vertex_cloud.points[3]))/right_line_real_length;
    error[3] = fabs(right_line_real_length - pcl::euclideanDistance(vertex_cloud.points[3], vertex_cloud.points[0]))/right_line_real_length;

    ROS_INFO("error = (%f, %f, %f, %f)", error[0], error[1], error[2], error[3]);
    if (error[0] <= error_thresh && error[1] <= error_thresh && error[2] <= error_thresh && error[3] <= error_thresh)
    {
		  writeData();
    }
		clicked_velodyne_points_.clear();
    
    printf("filter x = [%f, %f], y = [%f, %f], z = [%f, %f] \n", filter_xmin, filter_xmax, filter_ymin, filter_ymax, filter_zmin, filter_zmax);
	}



}



int main(int argc, char** argv)
{
    //subscriber_clicked_point_ = node_handle_.subscribe("/clicked_point", 1, &ROSCameraLidarApp::RvizClickedPointCallback, this);
    ros::init(argc, argv,"velodyne_click_vertex");
    ros::NodeHandle 	node_handle_;
    ros::Subscriber     subscriber_clicked_point_;
    vertex_cloud.points.resize(4);
    
    subscriber_clicked_point_ = node_handle_.subscribe("/clicked_point", 1, &RvizClickedPointCallback);
    
    ros::Publisher vertex_pub;
    vertex_pub = node_handle_.advertise<PPointCloud>("/vertex_points",1);
    
    while(ros::ok())
    {
      vertex_pub.publish(vertex_cloud);
      ros::spinOnce();
    }
    
    return 0;
}
