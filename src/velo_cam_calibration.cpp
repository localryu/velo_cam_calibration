#include <velo_cam_calibration/velodyne_vertex.h>
#include <iostream>
#include <string>
#include <dirent.h>

void writeMatrix(float* trans_matrix)
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

    std::string path = ros::package::getPath("velo_cam_calibration");
    std::string backuppath = path + "/data/calibration_matrix_"+ str +".xml";
    path = path + "/data/calibration_matrix.xml";

    std::cout << std::endl << "Creating .xml file with vertices in: "<< std::endl << path.c_str() << std::endl;
    
    // Create .xml file with vertices
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "utf-8", "");
    doc.LinkEndChild( decl );
    TiXmlElement * root = new TiXmlElement( "Matrix" );
    doc.LinkEndChild( root );

    std::string name[12] = {"m11", "m12", "m13", "m14", "m21", "m22", "m23", "m24", "m31", "m32", "m33", "m34"};
    TiXmlElement * mat = new TiXmlElement( "M" );
    root->LinkEndChild( mat );
    for (int i = 0;i<12;i++)
    {
        std::ostringstream sstream;
        sstream << trans_matrix[i];
        std::string m = sstream.str();
        mat->SetAttribute(name[i],m);
        
    }
        
    // Save XML file and copy
    doc.SaveFile(path);
    doc.SaveFile(backuppath);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velo_cam_calibration");

    std::vector<pcl::PointXYZ> lidar_pts;
    std::vector<cv::Point> cam_pts;

    ROS_INFO("Load xml file");
    std::string path = ros::package::getPath("velo_cam_calibration");

    std::vector<std::string> velo_filenames;
    std::vector<std::string> cam_filenames;

    // get name of .xml
    DIR *dir;
    struct dirent *ent;
    std::string datapath = path+"/data/";
    const char* src = datapath.c_str();
    if ((dir = opendir (src)) != NULL)
    {
        while ((ent = readdir (dir)) != NULL)
        {
            std::string tmp(ent->d_name);
            if (tmp.find("vertices_") != std::string::npos)
            {
                velo_filenames.push_back(tmp);
            }
            else if (tmp.find("camera_") != std::string::npos)
            {
                cam_filenames.push_back(tmp);
            }
            
        }
        closedir(dir);
    }
    else
    {
        std::cout << "fail" << std::endl;
    }
    
    std::sort(velo_filenames.begin(), velo_filenames.end());
    std::sort(cam_filenames.begin(), cam_filenames.end());

    for (int i = 0;i<velo_filenames.size();i++)
    {
        std::cout << velo_filenames[i] << std::endl;
    }
    for (int i = 0;i<cam_filenames.size();i++)
    {
        std::cout << cam_filenames[i] << std::endl;
    }
    std::cout << "-------------------------" << std::endl;
    
    // Read vertex coordi on velodyne
    for (int i = 0;i<velo_filenames.size();i++)
    {
        TiXmlDocument velo_doc;
        velo_doc.LoadFile(datapath + velo_filenames[i]);
        TiXmlElement* root = velo_doc.FirstChildElement("Vertices");
        std::string name[4] = {"VU", "VL", "VD", "VR"};
        TiXmlElement* sub;
        for (int j = 0;j<4;j++)
        {
            sub = root->FirstChildElement(name[j]);
            //std::cout << tmp->Value() << std::endl;
            std::string str = sub->Attribute("x");
            float x = stof(str);
            str = sub->Attribute("y");
            float y = stof(str);
            str = sub->Attribute("z");
            float z = stof(str);
            lidar_pts.push_back(pcl::PointXYZ(x,y,z));
            
        }
    }

    // Read vertex coordi on camera
    for (int i = 0;i<cam_filenames.size();i++)
    {
        TiXmlDocument velo_doc;
        velo_doc.LoadFile(datapath + cam_filenames[i]);
        TiXmlElement* root = velo_doc.FirstChildElement("Vertices");
        std::string name[4] = {"VU", "VL", "VD", "VR"};
        TiXmlElement* sub;
        for (int j = 0;j<4;j++)
        {
            sub = root->FirstChildElement(name[j]);
            //std::cout << tmp->Value() << std::endl;
            std::string str = sub->Attribute("x");
            int x = stoi(str);
            str = sub->Attribute("y");
            int y = stoi(str);
            cam_pts.push_back(cv::Point(x,y));
            
        }
    }
    
    
    for (int i = 0;i<lidar_pts.size();i++)
    {
        std::cout << lidar_pts[i].x << " " << lidar_pts[i].y << " " << lidar_pts[i].z << std::endl;
    }
    std::cout << "-------------------------" << std::endl;
    for (int i = 0;i<cam_pts.size();i++)
    {
        std::cout << cam_pts[i].x << " " << cam_pts[i].y << std::endl;
    }
    
    if (lidar_pts.size() != cam_pts.size())
    {
        std::cout << "num of lidar_pts is not same num of cam_pts" << std::endl;
        return 0;
    }
    int cali_ptrs = lidar_pts.size();
    std::cout <<"num of pts: " << cali_ptrs << std::endl;
    

    // SVD
    Eigen::MatrixXf pt_pair_matrix = Eigen::MatrixXf::Random(cali_ptrs*2,12);
    for (int i = 0;i<cali_ptrs;i++)
    {
        pt_pair_matrix(2*i, 0) = lidar_pts[i].x;
        pt_pair_matrix(2*i, 1) = lidar_pts[i].y;
        pt_pair_matrix(2*i, 2) = lidar_pts[i].z;
        pt_pair_matrix(2*i, 3) = 1;
        pt_pair_matrix(2*i, 4) = 0;
        pt_pair_matrix(2*i, 5) = 0;
        pt_pair_matrix(2*i, 6) = 0;
        pt_pair_matrix(2*i, 7) = 0;
        pt_pair_matrix(2*i, 8) = -1*cam_pts[i].x*lidar_pts[i].x;
        pt_pair_matrix(2*i, 9) = -1*cam_pts[i].x*lidar_pts[i].y;
        pt_pair_matrix(2*i, 10) = -1*cam_pts[i].x*lidar_pts[i].z;
        pt_pair_matrix(2*i, 11) = -1*cam_pts[i].x;

        pt_pair_matrix(2*i+1, 0) = 0;
        pt_pair_matrix(2*i+1, 1) = 0;
        pt_pair_matrix(2*i+1, 2) = 0;
        pt_pair_matrix(2*i+1, 3) = 0;
        pt_pair_matrix(2*i+1, 4) = lidar_pts[i].x;
        pt_pair_matrix(2*i+1, 5) = lidar_pts[i].y;
        pt_pair_matrix(2*i+1, 6) = lidar_pts[i].z;
        pt_pair_matrix(2*i+1, 7) = 1;
        pt_pair_matrix(2*i+1, 8) = -1*cam_pts[i].y*lidar_pts[i].x;
        pt_pair_matrix(2*i+1, 9) = -1*cam_pts[i].y*lidar_pts[i].y;
        pt_pair_matrix(2*i+1, 10) = -1*cam_pts[i].y*lidar_pts[i].z;
        pt_pair_matrix(2*i+1, 11) = -1*cam_pts[i].y;
    }
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(pt_pair_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    // transformation matrix
    float trans_matrix[12];
    for(int i=0; i<12; i++)
    {
        trans_matrix[i] = svd.matrixV()(i,11);
    }
    for (int i = 0;i<3;i++)
    {
        ROS_INFO("%f %f %f %f", trans_matrix[4*i], trans_matrix[4*i+1], trans_matrix[4*i+2],trans_matrix[4*i+3]);
    }
    writeMatrix(trans_matrix);
    for (int i = 0;i<lidar_pts.size();i++)
    {
        float Xl_L = lidar_pts[i].x;
        float Yl_L = lidar_pts[i].y;
        float Zl_L = lidar_pts[i].z;
        int xp = ((trans_matrix[0]*Xl_L + trans_matrix[1]*Yl_L + trans_matrix[2]*Zl_L + trans_matrix[3])/(trans_matrix[8]*Xl_L + trans_matrix[9]*Yl_L + trans_matrix[10]*Zl_L + trans_matrix[11]));
        int yp = ((trans_matrix[4]*Xl_L + trans_matrix[5]*Yl_L + trans_matrix[6]*Zl_L + trans_matrix[7])/(trans_matrix[8]*Xl_L + trans_matrix[9]*Yl_L + trans_matrix[10]*Zl_L + trans_matrix[11]));
        printf("(xp,yp) = (%d, %d)\n", xp,yp);
    }
    return 0;
}