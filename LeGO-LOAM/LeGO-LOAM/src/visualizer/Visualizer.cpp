//
// Created by hyye on 4/7/18.
//

#include "visualizer/Visualizer.h"

Visualizer::Visualizer(std::string vis_name,
                       std::vector<double> imu_color,
                       std::vector<double> lidar_color) 
{
//   // initial
//   vis_name_ = vis_name;
//   nh_ = ros::NodeHandle(vis_name_);
//   imu_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("imu_markers", 10);
//   lidar_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lidar_markers", 10);
//   velocity_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("velocity_marker", 10);
// 
// //  un_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/un_imu/data", 10);
// //  vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_test", 10);
// 
//   imu_marker_.header.frame_id = "/world";
//   imu_marker_.header.stamp = ros::Time();
//   imu_marker_.ns = "imu_marker";
//   imu_marker_.type = visualization_msgs::Marker::ARROW;
//   imu_marker_.action = visualization_msgs::Marker::ADD;
//   imu_marker_.pose.position.x = 0;
//   imu_marker_.pose.position.y = 0;
//   imu_marker_.pose.position.z = 0;
//   imu_marker_.pose.orientation.x = 0.0;
//   imu_marker_.pose.orientation.y = 0.0;
//   imu_marker_.pose.orientation.z = 0.0;
//   imu_marker_.pose.orientation.w = 1.0;
//   imu_marker_.scale.x = 0.2;
//   imu_marker_.scale.y = 0.05;
//   imu_marker_.scale.z = 0.05;
//   imu_marker_.color.a = 1.0; // Don't forget to set the alpha!
//   imu_marker_.color.r = imu_color[0];
//   imu_marker_.color.g = imu_color[1];
//   imu_marker_.color.b = imu_color[2];
// 
//   lidar_marker_.header.frame_id = "/world";
//   lidar_marker_.header.stamp = ros::Time();
//   lidar_marker_.ns = "lidar_markers";
//   lidar_marker_.type = visualization_msgs::Marker::ARROW;
//   lidar_marker_.action = visualization_msgs::Marker::ADD;
//   lidar_marker_.pose.position.x = 0;
//   lidar_marker_.pose.position.y = 0;
//   lidar_marker_.pose.position.z = 0;
//   lidar_marker_.pose.orientation.x = 0.0;
//   lidar_marker_.pose.orientation.y = 0.0;
//   lidar_marker_.pose.orientation.z = 0.0;
//   lidar_marker_.pose.orientation.w = 1.0;
//   lidar_marker_.scale.x = 0.2;
//   lidar_marker_.scale.y = 0.05;
//   lidar_marker_.scale.z = 0.05;
//   lidar_marker_.color.a = 1.0; // Don't forget to set the alpha!
//   lidar_marker_.color.r = lidar_color[0];
//   lidar_marker_.color.g = lidar_color[1];
//   lidar_marker_.color.b = lidar_color[2];
// 
//   velocity_marker_.header.frame_id = "/world";
//   velocity_marker_.header.stamp = ros::Time();
//   velocity_marker_.ns = "velocity_marker";
//   velocity_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//   velocity_marker_.action = visualization_msgs::Marker::ADD;
//   velocity_marker_.pose.position.x = 0;
//   velocity_marker_.pose.position.y = 0;
//   velocity_marker_.pose.position.z = 0;
//   velocity_marker_.pose.orientation.x = 0.0;
//   velocity_marker_.pose.orientation.y = 0.0;
//   velocity_marker_.pose.orientation.z = 0.0;
//   velocity_marker_.pose.orientation.w = 1.0;
//   velocity_marker_.scale.x = 5;
//   velocity_marker_.scale.y = 5;
//   velocity_marker_.scale.z = 5;
//   velocity_marker_.color.a = 1.0; // Don't forget to set the alpha!
//   velocity_marker_.color.r = 1;
//   velocity_marker_.color.g = 1;
//   velocity_marker_.color.b = 0;
}

PlaneNormalVisualizer::PlaneNormalVisualizer()
{
}

/**
 * @brief ...
 * 
 * @param cloud1_full ref full cloud
 * @param cloud2_full data full cloud
 * @param cloud1 ref coorrespondences
 * @param cloud2 data coorrespondences
 * @param line_color 
 * @return void
 */
void PlaneNormalVisualizer::UpdateLines(pcl::PointCloud<PointType>::ConstPtr cloud1_full,
										pcl::PointCloud<PointType>::ConstPtr cloud2_full,
										pcl::PointCloud<PointType>::ConstPtr cloud1,
                                        pcl::PointCloud<PointType>::ConstPtr cloud2,
                                        std::vector<double> line_color) 
{
	boost::mutex::scoped_lock lk(m);

	//  LOG(INFO) << ">>>>>>> update <<<<<<<";

	int num_cloud1 = cloud1->size();
	int num_cloud2 = cloud2->size();
	if (num_cloud1 == 0 || num_cloud2 == 0 || num_cloud1 != num_cloud2) 
	{
		LOG(INFO) << ">>>>>>> no points or sizes are not the same <<<<<<<";
		LOG_IF(INFO, num_cloud1 != num_cloud2) << num_cloud1 << " != " << num_cloud2;
		return;
	}

	for (const auto line_name : line_names) 
	{
		viewer->removeShape(line_name);
	}

	line_names.clear();

	pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud1_full, 255, 255, 255);
	viewer->addPointCloud<PointType>(cloud1_full, single_color, "ref_surf_cloud_full");
	single_color = pcl::visualization::PointCloudColorHandlerCustom<PointType>(cloud2_full, 0, 0, 255);
	viewer->addPointCloud<PointType>(cloud2_full, single_color, "data_surf_cloud_full");
// 	single_color = pcl::visualization::PointCloudColorHandlerCustom<PointType>(cloud1, 255, 255, 255);
// 	viewer->addPointCloud<PointType>(cloud1, single_color, "ref_surf_cloud");
// 	single_color = pcl::visualization::PointCloudColorHandlerCustom<PointType>(cloud2, 0, 0, 255);
// 	viewer->addPointCloud<PointType>(cloud2, single_color, "data_surf_cloud");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "ref_surf_cloud_full");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "data_surf_cloud_full");	
// 	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "ref_surf_cloud");
// 	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "data_surf_cloud");
	
	for (int i = 0; i < num_cloud1; ++i) 
	{
		std::stringstream line_name_ss;
		line_name_ss << "line" << i;
		std::string line_name = line_name_ss.str();

		viewer->addLine(cloud1->at(i), cloud2->at(i), line_name);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
											line_color[0],
											line_color[1],
											line_color[2],
											line_name);
		line_names.push_back(line_name);
	}
}

void PlaneNormalVisualizer::Spin() 
{
	{
		boost::mutex::scoped_lock lk(m);
		viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Debug Viewer");
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addCoordinateSystem(1.0);
		viewer->addText("debugger by Kitkat7", 10, 10, "debugger text", 0);
		viewer->initCameraParameters();
		init = true;
	}

	while (!viewer->wasStopped()) 
	{
		{
		boost::mutex::scoped_lock lk(m);
	//      LOG(INFO) << ">>>>>>> spin <<<<<<<";
		viewer->spinOnce(100);
		}
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
}


