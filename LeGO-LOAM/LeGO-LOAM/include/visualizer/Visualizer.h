//
// Created by hyye on 4/7/18.
//

#ifndef PLLO_VISUALIZER_H_
#define PLLO_VISUALIZER_H_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <map>

#include <iostream>

#include <glog/logging.h> // tutorial: http://rpg.ifi.uzh.ch/docs/glog.html

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZI  PointType;

class Visualizer 
{
public:
	Visualizer(std::string vis_name = "visualizer",
				std::vector<double> imu_color = {0.0, 1.0, 0.0},
				std::vector<double> lidar_color = {1.0, 1.0, 0.0});

	//   void UpdateMarker(visualization_msgs::Marker &marker, const Transform &pose);
	//   void UpdateMarkers(std::vector<Transform> imu_poses, std::vector<Transform> lidar_poses);
	//   void UpdateVelocity(double velocity);
	//   void PublishMarkers();
	// 
	//   ros::NodeHandle nh_;
	//   ros::Publisher imu_vis_pub_, lidar_vis_pub_, velocity_vis_pub_;
	// 
	//   tf::TransformBroadcaster br_;
	//   visualization_msgs::Marker imu_marker_, lidar_marker_, velocity_marker_;
	// 
	//   visualization_msgs::MarkerArray imu_markers_;
	//   visualization_msgs::MarkerArray lidar_markers_;
	// 
	//   std::string vis_name_;
};

class PlaneNormalVisualizer
{
public:
	PlaneNormalVisualizer();
	void Spin();
	void UpdateCloud(pcl::PointCloud<PointType>::ConstPtr cloud,
					std::string cloud_name = "cloud`",
					std::vector<double> cloud_color = {1.0, 0.0, 1.0});

	void UpdateCloudAndNormals(pcl::PointCloud<PointType>::ConstPtr cloud,
								pcl::PointCloud<pcl::Normal>::ConstPtr normals,
								int ds_ratio = 10,
								std::string cloud_name = "cloud",
								std::string normals_name = "normals",
								std::vector<double> cloud_color = {1.0, 1.0, 1.0},
								std::vector<double> normals_color = {1.0, 1.0, 0.0});

	void UpdateLines(pcl::PointCloud<PointType>::ConstPtr cloud1_full,
					pcl::PointCloud<PointType>::ConstPtr cloud2_full,
					pcl::PointCloud<PointType>::ConstPtr cloud1,
                    pcl::PointCloud<PointType>::ConstPtr cloud2,
					std::vector<double> line_color = {1.0, 0.0, 0.0});

	void UpdatePlanes(const std::vector<Eigen::Vector4d,
										Eigen::aligned_allocator<Eigen::Vector4d>> &plane_coeffs);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//  pcl::visualization::PCLVisualizer* viewer;
	boost::mutex m;
	bool init = false;
	bool first = false;

	std::vector<std::string> line_names;
	std::vector<std::string> plane_names;
};


#endif //PLLO_VISUALIZER_H_