cmake_minimum_required(VERSION 2.8.3)
project(velo_cam_calibration)

find_package(OpenCV REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(catkin REQUIRED 
                cmake_modules
                COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS}
             angles
             pcl_conversions
             roslaunch rostest
	     roscpp
	     rospy
	     std_msgs
             roslib)

find_package(TinyXML REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost)
find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(PCL 1.7 REQUIRED)


add_definitions(-std=c++11)

set(${PROJECT_NAME}_CATKIN_DEPS
    nodelet
    roscpp
    pcl_ros
    sensor_msgs
    tf
    std_msg
    cv_bridge
    image_transport)

include_directories(include
                    #/home/ryu/catkin_ws/src/velodyne/velodyne_pointcloud/include
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIR}
		    ${OpenCV_INCLUDE_DIRS}
                    ${TinyXML_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS} roslib DEPENDS TinyXML)

add_executable(velodyne_vertex_node src/velodyne_vertex_node.cpp src/velodyne_vertex.cpp)
target_link_libraries(velodyne_vertex_node
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${TinyXML_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${Eigen_LIBRARIES})

add_executable(velodyne_click_vertex src/velodyne_click_vertex.cpp)
target_link_libraries(velodyne_click_vertex
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${TinyXML_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${Eigen_LIBRARIES})

add_executable(camera_click_vertex src/camera_click_vertex.cpp)
target_link_libraries(camera_click_vertex
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${TinyXML_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${Eigen_LIBRARIES})

# install the files needed to run this package
install(TARGETS velodyne_vertex_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(velo_cam_calibration src/velo_cam_calibration.cpp)
target_link_libraries(velo_cam_calibration
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${TinyXML_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${Eigen_LIBRARIES})

# install the files needed to run this package
install(TARGETS velo_cam_calibration
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

