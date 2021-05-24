/*  Copyright (C) 2010 UT-Austin &  Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 *  Copyright (C) 2012 Jack O'Quin
 *
 *  License: Modified BSD Software License
 */

/** @file

    @brief ROS class for detecting obstacles in a point cloud.

   This class produces a point cloud containing all points that lie on
   an obstacle and are taller than the @c height_threshold parameter.

Subscribes:

- @b velodyne_points [sensor_msgs::PointCloud2] data from one
  revolution of the Velodyne LIDAR

Publishes:

- @b veloydne_obstacles [sensor_msgs::PointCloud2] grid cells that
  contain an obstacle

- @b veloydne_clear [sensor_msgs::PointCloud2] grid cells with no
  obstacles


@author David Claridge, Michael Quinlan

*/

#include <velo_cam_calibration/velodyne_vertex.h>


Velodyne::Velodyne(ros::NodeHandle node, ros::NodeHandle priv_nh)
{
  // get parameters using private node handle
  priv_nh.param("scan_line", _scan_line, 32);
  ROS_INFO("scan_line: %d", _scan_line);
  priv_nh.param("click_filter_pts", _click_filter_pts, false);
  ROS_INFO("click_filter_pts: %d", _click_filter_pts);

  priv_nh.param("filter_xmin", _filter_xmin, 2.2);
  ROS_INFO("filter_xmin: %f", _filter_xmin);
  priv_nh.param("filter_xmax", _filter_xmax, 2.5);
  ROS_INFO("filter_xmax: %f", _filter_xmax);

  priv_nh.param("filter_ymin", _filter_ymin, -1.7);
  ROS_INFO("filter_ymin: %f", _filter_ymin);
  priv_nh.param("filter_ymax", _filter_ymax, -0.4);
  ROS_INFO("filter_ymax: %f", _filter_ymax);

  priv_nh.param("filter_zmin", _filter_zmin, -1.0);
  ROS_INFO("filter_zmin: %f", _filter_zmin);
  priv_nh.param("filter_zmax", _filter_zmax, 0.3);
  ROS_INFO("filter_zmax: %f", _filter_zmax);

  priv_nh.param("distance_thresh", _distance_thresh, 0.01);
  ROS_INFO("distance_thresh: %f", _distance_thresh);
  priv_nh.param("error_thresh", _error_thresh, 0.07);
  ROS_INFO("error_thresh: %f", _error_thresh);

  priv_nh.param("left_line_real_length", _left_line_real_length, 0.905);
  ROS_INFO("left_line_real_length: %f", _left_line_real_length);
  priv_nh.param("right_line_real_length", _right_line_real_length, 0.905);
  ROS_INFO("right_line_real_length: %f", _right_line_real_length);

  start_ring = _scan_line - 1;
  end_ring = 0;

  // Set up publishers
  xfiltered_publisher = node.advertise<VPointCloud>("xfiltered_points",1);
  xyfiltered_publisher = node.advertise<VPointCloud>("xyfiltered_points",1);
  xyzfiltered_publisher = node.advertise<VPointCloud>("xyzfiltered_points",1);

  projected_publisher = node.advertise<VPointCloud>("projected_points",1);
  lines_publisher = node.advertise<RGBPointCloud>("line_points",1);

  vertex_publisher = node.advertise<RGBPointCloud>("vertex_points",1);

  // subscribe to Velodyne data points
  velodyne_scan = node.subscribe("/velodyne_points", 10,
                                  &Velodyne::processData, this,
                                  ros::TransportHints().tcpNoDelay(true));
  // velodyne_scan = node.subscribe("/lidar_filter/cloud_filtered", 10,
  //                                 &Velodyne::processData, this,
  //                                 ros::TransportHints().tcpNoDelay(true));
  if (_click_filter_pts)
    subscriber_clicked_point_ = node.subscribe("/clicked_point", 1, &Velodyne::RvizClickedPointCallback, this);
}

Velodyne::~Velodyne() {}

void Velodyne::RvizClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point)
{
	clicked_velodyne_points_.push_back(cv::Point3f(in_clicked_point.point.x,
	                                               in_clicked_point.point.y,
	                                               in_clicked_point.point.z));

	std::cout << cv::Point3f(in_clicked_point.point.x,
	                         in_clicked_point.point.y,
	                         in_clicked_point.point.z) << std::endl << std::endl;
	std::cout << "Number of points: " << clicked_velodyne_points_.size() << std::endl;

	filter_xmin = MIN(in_clicked_point.point.x, filter_xmin);
  filter_xmax = MAX(in_clicked_point.point.x, filter_xmax);
  filter_ymin = MIN(in_clicked_point.point.y, filter_ymin);
  filter_ymax = MAX(in_clicked_point.point.y, filter_ymax);
  filter_zmin = MIN(in_clicked_point.point.z, filter_zmin);
  filter_zmax = MAX(in_clicked_point.point.z, filter_zmax);
  if (clicked_velodyne_points_.size() == 4)
	{
    _filter_xmin = filter_xmin - 0.05;
    _filter_xmax = filter_xmax + 0.05;
    _filter_ymin = filter_ymin - 0.05;
    _filter_ymax = filter_ymax + 0.05;
    _filter_zmin = filter_zmin - 0.05;
    _filter_zmax = filter_zmax + 0.05;
    ROS_INFO("filter x = [%f, %f], y = [%f, %f], z = [%f, %f]", _filter_xmin, _filter_xmax, _filter_ymin, _filter_ymax, _filter_zmin, _filter_zmax);
    _click_filter_pts = false;
  }

}

void Velodyne::initializeCloud(const VPointCloud::ConstPtr &scan)
{
  num_pts_on_ring.resize(_scan_line);
  projected_plane_cloud.header = scan->header;
  UL_line.header = scan->header;
  UR_line.header = scan->header;
  DL_line.header = scan->header;
  DR_line.header = scan->header;
  vertex_cloud.header = scan->header;
  vertex_cloud.resize(4);
}

void Velodyne::lineFitting(const PPointCloud::Ptr &line_cloud, Eigen::VectorXf &coefficients_vec)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PPoint> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (line_cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    ROS_WARN("[Laser] Could not estimate a planar model for the given dataset.");
    return;
  }

  // Copy coefficients to proper object for further filtering
  coefficients_vec(0) = coefficients->values[0];
  coefficients_vec(1) = coefficients->values[1];
  coefficients_vec(2) = coefficients->values[2];
  coefficients_vec(3) = coefficients->values[3];
  coefficients_vec(4) = coefficients->values[4];
  coefficients_vec(5) = coefficients->values[5];
}

void Velodyne::planeFitting(const VPointCloud::ConstPtr &scan, float &A, float &B, float &C, float &D)
{
  float max_inlier_ratio = 0.;
  bool* bCheckExistOfNum = (bool*)malloc(filtered_count*sizeof(bool));
  memset(bCheckExistOfNum, 0, filtered_count*sizeof(bool));

  // iteration: 100
  srand((unsigned)time(NULL));
  int random_count = 0;
  for (int i = 0;i<100;i++)
  {
    // pick 3 random numbers
    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    //int rand_index;

    unsigned int rand_index[3];
    for (int j = 0;j<3;j++)
    {
      while (1)
      {
        unsigned int random_num = rand() % filtered_count;
        if (bCheckExistOfNum[random_num]==false)
        {
          bCheckExistOfNum[random_num]=true;
          rand_index[j] = random_num;
          random_count++;
          break;
        }
        // all of points are already used
        if (random_count == filtered_count)
        {
          break;
        }
      }
    }

    x1 = scan->points[rand_index[0]].x;
    y1 = scan->points[rand_index[0]].y;
    z1 = scan->points[rand_index[0]].z;

    x2 = scan->points[rand_index[1]].x;
    y2 = scan->points[rand_index[1]].y;
    z2 = scan->points[rand_index[1]].z;

    x3 = scan->points[rand_index[2]].x;
    y3 = scan->points[rand_index[2]].y;
    z3 = scan->points[rand_index[2]].z;

    // http://www.gisdeveloper.co.kr/?p=801
    // define plane by using 3 random numbers
    // ax + by + cz + d = 0
    float a = y1*(z2-z3) + y2*(z3-z1) + y3*(z1-z2);
    float b = z1*(x2-x3) + z2*(x3-x1) + z3*(x1-x2);
    float c = x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2);
    float d = -x1*(y2*z3-y3*z2) - x2*(y3*z1-y1*z3) - x3*(y1*z2-y2*z1);


    // find the plane that have the maximum of inlier line ratio
    float sum_of_wI = 0.; // assume wn = 1/(num of ptrs whose ring number is n)
    for (int j = 0;j<filtered_count;j++)
    {
      float x = scan->points[j].x;
      float y = scan->points[j].y;
      float z = scan->points[j].z;
      float ring = scan->points[j].ring;
      float dist = fabs(a*x + b*y + c*z + d)/sqrt(a*a + b*b + c*c);

      if (dist <= _distance_thresh)
        sum_of_wI += 1./(num_pts_on_ring[ring]);
    }
    // calculate inlier_line_ratio
    float inlier_line_ratio = sum_of_wI/num_of_rings;
    if (inlier_line_ratio > max_inlier_ratio)
    {
      max_inlier_ratio = inlier_line_ratio;
      A = a, B = b, C = c, D = d;
    }

  }
  free(bCheckExistOfNum);
}

void Velodyne::projectOnPlane(const VPointCloud::ConstPtr &scan, float &A, float &B, float &C, float &D)
{
  // project points to plane
  for (int i = 0;i<filtered_count;i++)
  {
    float x = scan->points[i].x;
    float y = scan->points[i].y;
    float z = scan->points[i].z;
    float ring = scan->points[i].ring;
    float t = -(A*x + B*y + C*z + D)/(A*A + B*B + C*C);

    projected_plane_cloud.points[i].x = A*t + x;
    projected_plane_cloud.points[i].y = B*t + y;
    projected_plane_cloud.points[i].z = C*t + z;
    projected_plane_cloud.points[i].ring = ring;
    ring_projected_cloud[ring - start_ring].header.stamp = scan->header.stamp;
    ring_projected_cloud[ring - start_ring].header.frame_id = scan->header.frame_id;
    ring_projected_cloud[ring - start_ring].points.push_back(projected_plane_cloud.points[i]);
  }
}

void Velodyne::writeData(Eigen::Vector4f* V)
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

  ROS_INFO("VU = (%.4f, %.4f, %.4f)",V[0](0),V[0](1),V[0](2));
  ROS_INFO("VL = (%.4f, %.4f, %.4f)",V[1](0),V[1](1),V[1](2));
  ROS_INFO("VD = (%.4f, %.4f, %.4f)",V[2](0),V[2](1),V[2](2));
  ROS_INFO("VD = (%.4f, %.4f, %.4f)",V[3](0),V[3](1),V[3](2));

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
    sstreamx << V[i](0);
    sstreamy << V[i](1);
    sstreamz << V[i](2);
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


/** point cloud input callback */
void Velodyne::processData(const VPointCloud::ConstPtr &scan)
{
  if (_click_filter_pts)
    return;

  bool flip = false;
  bool xaxis_ref = false;

  if (fabs(_filter_xmax - _filter_xmin) > fabs(_filter_ymax - _filter_ymin))
    xaxis_ref = true;

  VPointCloud xfiltered_cloud, xyfiltered_cloud, xyzfiltered_cloud;
  xfiltered_cloud.header = scan->header;
  xyfiltered_cloud.header = scan->header;
  xyzfiltered_cloud.header = scan->header;

  // filtering
  for (int i = 0;i<scan->points.size();i++)
  {
    VPoint tmp = scan->points[i];
    if (tmp.x >= _filter_xmin && tmp.x <= _filter_xmax)
    {
      xfiltered_cloud.points.push_back(tmp);
      if (tmp.y >= _filter_ymin && tmp.y <= _filter_ymax)
      {
        xyfiltered_cloud.points.push_back(tmp);
        if (tmp.z >= _filter_zmin && tmp.z <= _filter_zmax)
          xyzfiltered_cloud.points.push_back(tmp);
      }
    }
  }
  xfiltered_publisher.publish(xfiltered_cloud);
  xyfiltered_publisher.publish(xyfiltered_cloud);
  xyzfiltered_publisher.publish(xyzfiltered_cloud);

  filtered_count = xyzfiltered_cloud.points.size();
  VPointCloud::Ptr filtered_cloud (new VPointCloud);
  copyPointCloud(xyzfiltered_cloud, *filtered_cloud);

  // initialize header and fixed size
  initializeCloud(filtered_cloud);

  // get num of points in each ring
  for (int i = 0;i<filtered_count;i++)
  {
    unsigned int ring = filtered_cloud->points[i].ring;
    num_pts_on_ring[ring]++;
    if (ring < start_ring)
        start_ring = ring;
    if (ring > end_ring)
        end_ring = ring;
  }
  ROS_INFO("ring = [%d, %d]", start_ring, end_ring);
  // number of rings
  num_of_rings = end_ring - start_ring + 1;

  ROS_INFO("size = %d", filtered_count);

  // find plane equation by using RANSAC
  float A, B, C, D;
  planeFitting(filtered_cloud, A, B, C, D);
  // projected point cloud on plane
  projected_plane_cloud.resize(filtered_count);

  // ring_projected_cloud[ring_num]: projected point cloud whose ring number is (ring_num + start_ring)
  ring_projected_cloud.resize(num_of_rings);

  projectOnPlane(filtered_cloud, A, B, C, D);

  projected_publisher.publish(projected_plane_cloud);

  ////////// xaxis ref ///////////
  if (xaxis_ref)
  {
    // find the ring number have xmin
    // find the ring number have xmax
    // initialization
    int ring_of_xmin = -1;
    int ring_of_xmax = -1;
    float xmin = 10000.;
    float xmax = -10000.;
    float xmin_y = 0.;
    float xmax_y = 0.;
    for (int i = 0;i<num_of_rings;i++)
    {
      if (ring_projected_cloud[i].points.size() == 0)
      {
        ROS_INFO("filter is not good");
        return;
      }

      // find xmin and xmax of each ring
      int num_pts_of_ring = ring_projected_cloud[i].size();
      float xmin_of_ring = 10000.;
      float xmax_of_ring = -10000.;

      float xmin_y_of_ring = 10000.;
      float xmax_y_of_ring = -10000.;

      for (int j = 0;j<num_pts_of_ring;j++)
      {
        float x = ring_projected_cloud[i].points[j].x;
        float y = ring_projected_cloud[i].points[j].y;
        if (x < xmin_of_ring)
        {
          xmin_of_ring = x;
          xmin_y_of_ring = y;
        }
        if (x > xmax_of_ring)
        {
          xmax_of_ring = x;
          xmax_y_of_ring = y;
        }

      }

      // find xmin and xmax about all rings
      if (xmin_of_ring < xmin)
      {
        xmin = xmin_of_ring;
        xmin_y = xmin_y_of_ring;
        ring_of_xmin = i;
      }
      if (xmax_of_ring > xmax)
      {
        xmax = xmax_of_ring;
        xmax_y = xmax_y_of_ring;
        ring_of_xmax = i;
      }
    }

    if ((xmin_y + xmax_y)<0)
    {
      flip = true;
      int tmp = ring_of_xmin;
      ring_of_xmin = ring_of_xmax;
      ring_of_xmax = tmp;
    }
    ROS_INFO("x = (%f, %f), ring_of_x = (%d, %d)", xmin, xmax, ring_of_xmin, ring_of_xmax);

    // find points of up_left, up_right, down_left, down_right line
    for (int i = 0;i<num_of_rings;i++)
    {
      // VPoint xmin_tmp = ring_projected_cloud[i].points[0];
      // VPoint xmax_tmp = ring_projected_cloud[i].points[ring_projected_cloud[i].points.size() - 1];
      // find xmin and xmax of each ring
      int num_pts_of_ring = ring_projected_cloud[i].size();
      float xmin_of_ring = 10000.;
      float xmax_of_ring = -10000.;
      int xmin_idx = 0;
      int xmax_idx = 0;
      for (int j = 0;j<num_pts_of_ring;j++)
      {
        float x = ring_projected_cloud[i].points[j].x;
        if (x < xmin_of_ring)
        {
          xmin_of_ring = x;
          xmin_idx = j;
        }

        if (x > xmax_of_ring)
        {
          xmax_of_ring = x;
          xmax_idx = j;
        }

      }
      VPoint xmin_tmp = ring_projected_cloud[i].points[xmin_idx];
      VPoint xmax_tmp = ring_projected_cloud[i].points[xmax_idx];

      PPoint xmin_on_ring, xmax_on_ring;
      xmin_on_ring.x = xmin_tmp.x, xmin_on_ring.y = xmin_tmp.y, xmin_on_ring.z = xmin_tmp.z;
      xmax_on_ring.x = xmax_tmp.x, xmax_on_ring.y = xmax_tmp.y, xmax_on_ring.z = xmax_tmp.z;
      float dist_x = pcl::euclideanDistance(xmin_on_ring, xmax_on_ring);
      float delta_d = dist_x/ring_projected_cloud[i].points.size();
      //ROS_INFO("dist_x = %f, delta = %f", dist_x, delta_d);

      PPoint virtual_xmin = xmin_on_ring;
      PPoint virtual_xmax = xmax_on_ring;
      virtual_xmin.x -= delta_d/2*dist_x;
      virtual_xmax.x += delta_d/2*dist_x;

      // no flip
      if (!flip)
      {
        // down_left line
        if (i>=0 && i<=ring_of_xmin)
          DL_line.points.push_back(virtual_xmin);
        // up_left_line
        if (i>=ring_of_xmin && i<num_of_rings)
          UL_line.points.push_back(virtual_xmin);
        // down_right line
        if (i>=0 && i<=ring_of_xmax)
          DR_line.points.push_back(virtual_xmax);
        // up_right line
        if (i>=ring_of_xmax && i<num_of_rings)
          UR_line.points.push_back(virtual_xmax);
      }
      // flip
      else
      {
        // down_right line
        if (i>=0 && i<=ring_of_xmax)
          DR_line.points.push_back(virtual_xmin);
        // up_right_line
        if (i>=ring_of_xmax && i<num_of_rings)
          UR_line.points.push_back(virtual_xmin);
        // down_left line
        if (i>=0 && i<=ring_of_xmin)
          DL_line.points.push_back(virtual_xmax);
        // up_left line
        if (i>=ring_of_xmin && i<num_of_rings)
          UL_line.points.push_back(virtual_xmax);
      }
    }
  }
  ////////// xaxis ref ///////////

  ////////// yaxis ref ///////////
  else
  {
    // find the ring number have xmin
    // find the ring number have xmax
    // initialization
    int ring_of_ymin = -1;
    int ring_of_ymax = -1;
    float ymin = 10000.;
    float ymax = -10000.;
    float ymin_x = 0.;
    float ymax_x = 0.;
    for (int i = 0;i<num_of_rings;i++)
    {
      if (ring_projected_cloud[i].points.size() == 0)
      {
        ROS_INFO("filter is not good");
        return;
      }

      // find ymin and ymax of each ring
      int num_pts_of_ring = ring_projected_cloud[i].size();
      float ymin_of_ring = 10000.;
      float ymax_of_ring = -10000.;

      float ymin_x_of_ring = 10000.;
      float ymax_x_of_ring = -10000.;

      for (int j = 0;j<num_pts_of_ring;j++)
      {
        float x = ring_projected_cloud[i].points[j].x;
        float y = ring_projected_cloud[i].points[j].y;
        if (y < ymin_of_ring)
        {
          ymin_of_ring = y;
          ymin_x_of_ring = x;
        }
        if (y > ymax_of_ring)
        {
          ymax_of_ring = y;
          ymax_x_of_ring = x;
        }

      }

      // find ymin and ymax about all rings
      if (ymin_of_ring < ymin)
      {
        ymin = ymin_of_ring;
        ymin_x = ymin_x_of_ring;
        ring_of_ymin = i;
      }
      if (ymax_of_ring > ymax)
      {
        ymax = ymax_of_ring;
        ymax_x = ymax_x_of_ring;
        ring_of_ymax = i;
      }
    }

    if ((ymin_x + ymax_x)>0)
    {
      flip = true;
      int tmp = ring_of_ymin;
      ring_of_ymin = ring_of_ymax;
      ring_of_ymax = tmp;
    }
    ROS_INFO("y = (%f, %f), ring_of_y = (%d, %d)", ymin, ymax, ring_of_ymin, ring_of_ymax);

    // find points of up_left, up_right, down_left, down_right line
    for (int i = 0;i<num_of_rings;i++)
    {
      // find ymin and ymax of each ring
      int num_pts_of_ring = ring_projected_cloud[i].size();
      float ymin_of_ring = 10000.;
      float ymax_of_ring = -10000.;
      int ymin_idx = 0;
      int ymax_idx = 0;
      for (int j = 0;j<num_pts_of_ring;j++)
      {
        float y = ring_projected_cloud[i].points[j].y;
        if (y < ymin_of_ring)
        {
          ymin_of_ring = y;
          ymin_idx = j;
        }

        if (y > ymax_of_ring)
        {
          ymax_of_ring = y;
          ymax_idx = j;
        }

      }
      VPoint ymin_tmp = ring_projected_cloud[i].points[ymin_idx];
      VPoint ymax_tmp = ring_projected_cloud[i].points[ymax_idx];

      PPoint ymin_on_ring, ymax_on_ring;
      ymin_on_ring.x = ymin_tmp.x, ymin_on_ring.y = ymin_tmp.y, ymin_on_ring.z = ymin_tmp.z;
      ymax_on_ring.x = ymax_tmp.x, ymax_on_ring.y = ymax_tmp.y, ymax_on_ring.z = ymax_tmp.z;
      float dist_y = pcl::euclideanDistance(ymin_on_ring, ymax_on_ring);
      float delta_d = dist_y/ring_projected_cloud[i].points.size();
      //ROS_INFO("dist_y = %f, delta = %f", dist_y, delta_d);

      PPoint virtual_ymin = ymin_on_ring;
      PPoint virtual_ymax = ymax_on_ring;
      virtual_ymin.y -= delta_d/2*dist_y;
      virtual_ymax.y += delta_d/2*dist_y;

      // no flip
      if (!flip)
      {
        // down_left line
        if (i>=0 && i<=ring_of_ymin)
          DL_line.points.push_back(virtual_ymin);
        // up_left_line
        if (i>=ring_of_ymin && i<num_of_rings)
          UL_line.points.push_back(virtual_ymin);
        // down_right line
        if (i>=0 && i<=ring_of_ymax)
          DR_line.points.push_back(virtual_ymax);
        // up_right line
        if (i>=ring_of_ymax && i<num_of_rings)
          UR_line.points.push_back(virtual_ymax);
      }
      // flip
      else
      {
        // down_right line
        if (i>=0 && i<=ring_of_ymax)
          DR_line.points.push_back(virtual_ymin);
        // up_right_line
        if (i>=ring_of_ymax && i<num_of_rings)
          UR_line.points.push_back(virtual_ymin);
        // down_left line
        if (i>=0 && i<=ring_of_ymin)
          DL_line.points.push_back(virtual_ymax);
        // up_left line
        if (i>=ring_of_ymin && i<num_of_rings)
          UL_line.points.push_back(virtual_ymax);
      }
  }




  }


  // Find equation of line
  // [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z]
  Eigen::VectorXf coefficients_UL(6);
  lineFitting(UL_line.makeShared(), coefficients_UL);

  Eigen::VectorXf coefficients_UR(6);
  lineFitting(UR_line.makeShared(), coefficients_UR);

  Eigen::VectorXf coefficients_DL(6);
  lineFitting(DL_line.makeShared(), coefficients_DL);

  Eigen::VectorXf coefficients_DR(6);
  lineFitting(DR_line.makeShared(), coefficients_DR);


  RGBPointCloud lines;
  lines.header = scan->header;
  for (int i = 0;i<UL_line.points.size();i++)
  {
    RGBPoint tmp;
    tmp.x = UL_line.points[i].x, tmp.y = UL_line.points[i].y, tmp.z = UL_line.points[i].z;
    tmp.r = 255, tmp.g = 0, tmp.b = 0; //red
    lines.points.push_back(tmp);
  }
  for (int i = 0;i<UR_line.points.size();i++)
  {
    RGBPoint tmp;
    tmp.x = UR_line.points[i].x, tmp.y = UR_line.points[i].y, tmp.z = UR_line.points[i].z;
    tmp.r = 0, tmp.g = 255, tmp.b = 0; //green
    lines.points.push_back(tmp);
  }
  for (int i = 0;i<DL_line.points.size();i++)
  {
    RGBPoint tmp;
    tmp.x = DL_line.points[i].x, tmp.y = DL_line.points[i].y, tmp.z = DL_line.points[i].z;
    tmp.r = 0, tmp.g = 0, tmp.b = 255; //blue
    lines.points.push_back(tmp);
  }
  for (int i = 0;i<DR_line.points.size();i++)
  {
    RGBPoint tmp;
    tmp.x = DR_line.points[i].x, tmp.y = DR_line.points[i].y, tmp.z = DR_line.points[i].z;
    tmp.r = 255, tmp.g = 255, tmp.b = 0; //yellow
    lines.points.push_back(tmp);
  }
  lines_publisher.publish(lines);

  // Find intersection of two lines
  // Calculate VU, VL, VD, VR
  Eigen::Vector4f V[4]; // VU, VL, VD, VR
  pcl::lineWithLineIntersection(coefficients_UL, coefficients_UR, V[0]);
  pcl::lineWithLineIntersection(coefficients_UL, coefficients_DL, V[1]);
  pcl::lineWithLineIntersection(coefficients_DL, coefficients_DR, V[2]);
  pcl::lineWithLineIntersection(coefficients_UR, coefficients_DR, V[3]);

  // vertex cloud
  for (int i = 0;i<4;i++)
  {
    RGBPoint tmp;
    tmp.x = V[i](0), tmp.y = V[i](1), tmp.z = V[i](2);
    switch (i)
    {
      case 0:
        tmp.r = 255, tmp.g = 0, tmp.b = 0;
        break;
      case 1:
        tmp.r = 0, tmp.g = 255, tmp.b = 0;
        break;
      case 2:
        tmp.r = 0, tmp.g = 0, tmp.b = 255;
        break;
      case 3:
        tmp.r = 255, tmp.g = 255, tmp.b = 0;
        break;
      default:
        tmp.r = 255, tmp.g = 255, tmp.b = 255;
        break;
    }
    vertex_cloud.points[i] = tmp;
  }

  // calculate error
  float error[4];
  error[0] = fabs(_left_line_real_length - pcl::euclideanDistance(vertex_cloud.points[0], vertex_cloud.points[1]))/_left_line_real_length;
  error[1] = fabs(_left_line_real_length - pcl::euclideanDistance(vertex_cloud.points[1], vertex_cloud.points[2]))/_left_line_real_length;
  error[2] = fabs(_right_line_real_length - pcl::euclideanDistance(vertex_cloud.points[2], vertex_cloud.points[3]))/_right_line_real_length;
  error[3] = fabs(_right_line_real_length - pcl::euclideanDistance(vertex_cloud.points[3], vertex_cloud.points[0]))/_right_line_real_length;

  vertex_publisher.publish(vertex_cloud);
  ROS_INFO("error = (%f, %f, %f, %f)", error[0], error[1], error[2], error[3]);
  // write xml once
  if (write == false)
  {
    if (error[0] <= _error_thresh && error[1] <= _error_thresh && error[2] <= _error_thresh && error[3] <= _error_thresh)
    {
      writeData(V);
      write = true;
      //system("pkill roslaunch");
      return;
    }
  }


}
