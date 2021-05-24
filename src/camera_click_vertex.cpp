#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <time.h>
#include <math.h>

#include <ctime>
#include "tinyxml.h"

#include <ros/package.h>
#include <Eigen/Dense>
#include <vector>
#include <opencv2/highgui.hpp>  
#include <opencv2/imgproc.hpp>  

#include <boost/filesystem.hpp>
#include "boost/algorithm/string.hpp"

typedef std::vector<std::string> stringvec;
using namespace cv;

std::vector<cv::Point2i> clicked_image_points_;

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

  ROS_INFO("VU = (%d, %d)",clicked_image_points_[0].x,clicked_image_points_[0].y);
  ROS_INFO("VL = (%d, %d)",clicked_image_points_[1].x,clicked_image_points_[1].y);
  ROS_INFO("VD = (%d, %d)",clicked_image_points_[2].x,clicked_image_points_[2].y);
  ROS_INFO("VR = (%d, %d)",clicked_image_points_[3].x,clicked_image_points_[3].y);

  std::string path = ros::package::getPath("velo_cam_calibration");
  std::string backuppath = path + "/data/camera_"+ str +".xml";
  path = path + "/data/camera.xml";

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
    sstreamx << clicked_image_points_[i].x;
    sstreamy << clicked_image_points_[i].y;
    std::string Vx = sstreamx.str();
    std::string Vy = sstreamy.str();

    TiXmlElement * camera_node = new TiXmlElement( name[i] );
    root->LinkEndChild( camera_node );
    camera_node->SetAttribute("x",Vx);
    camera_node->SetAttribute("y",Vy);
  }
    
  // Save XML file and copy
  doc.SaveFile(path);
  doc.SaveFile(backuppath);
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        clicked_image_points_.push_back(Point2i(x,y));
        std::cout << clicked_image_points_.size() << "   (" << x << ", " << y << ")" << std::endl;
    }
    if (clicked_image_points_.size() == 4)
    {
      writeData();
		  clicked_image_points_.clear();
    }
}

struct path_leaf_string
{
    std::string operator()(const boost::filesystem::directory_entry& entry) const
    {
        return entry.path().leaf().string();
    }
};

int main(int argc, char** argv)
{
    stringvec v;
    stringvec img_files;
    std::string path = ros::package::getPath("velo_cam_calibration");
    std::string dir_path = path + "/data/";
    boost::filesystem::path p(dir_path);
    boost::filesystem::directory_iterator start(p);
    boost::filesystem::directory_iterator end;
    std::transform(start, end, std::back_inserter(v), path_leaf_string());
    std::string extension(".png");

    for (int i = 0;i<v.size();i++)
    {
      std::string filename = v[i];
      bool b = boost::contains(filename, extension);
      if (b)
        img_files.push_back(dir_path + filename);
    }

    std::sort(img_files.begin(), img_files.end());
    for (int i = 0;i<img_files.size();i++)
    {
      std::string img_filepath = img_files[i];
      
      Mat img;
    
      //이미지파일을 로드하여 image에 저장  
      img = imread(img_filepath, IMREAD_COLOR);
      if (img.empty())
      {
          std::cout << "Could not open or find the image" << std::endl;
          return -1;
      }
      std::string window_name = img_filepath;
      //윈도우 생성  
      namedWindow(window_name, WINDOW_AUTOSIZE);
  
      //윈도우에 출력  
      imshow(window_name, img);
  
      //윈도우에 콜백함수를 등록
      setMouseCallback(window_name, CallBackFunc, NULL);

      //키보드 입력이 될때까지 대기  
      int key = waitKey(0);
      if (key == 110)
      {
	      destroyWindow(window_name);
        continue;
      }
    }
      



    return 0;
}
