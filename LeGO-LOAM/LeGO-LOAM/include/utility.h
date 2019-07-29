#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "cloud_msgs/cloud_info.h"

#include <opencv/cv.h>

#include <eigen3/Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <boost/foreach.hpp>
#include <yaml-cpp/yaml.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#define PI 3.14159265

// #define foreach BOOST_FOREACH

using namespace std;

typedef pcl::PointXYZI  PointType;

// VLP-16
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

extern const bool loopClosureEnableFlag = false;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;
extern const string imuTopic = "/imu/data";


extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 1.0472;
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;

extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;

extern const float surroundingKeyframeSearchRadius = 50.0;
extern const int   surroundingKeyframeSearchNum = 50;

extern const float historyKeyframeSearchRadius = 5.0;
extern const int   historyKeyframeSearchNum = 25;
extern const float historyKeyframeFitnessScore = 0.3;

extern const float globalMapVisualizationSearchRadius = 500.0;


struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

void odom_to_matrix(const nav_msgs::Odometry &odom, Eigen::Matrix4d &tf)
{
	tf.setIdentity();
	Eigen::Quaterniond q(odom.pose.pose.orientation.w, 
						odom.pose.pose.orientation.x,
						odom.pose.pose.orientation.y,
						odom.pose.pose.orientation.z);
	tf.topLeftCorner(3, 3) = q.toRotationMatrix();
	tf(0, 3) = odom.pose.pose.position.x;
	tf(1, 3) = odom.pose.pose.position.y;
	tf(2, 3) = odom.pose.pose.position.z;
}

void matrix_to_odom(const Eigen::Matrix4d &tf, nav_msgs::Odometry &odom)
{
	Eigen::Matrix3d R = tf.topLeftCorner(3, 3);
	Eigen::Quaterniond q = Eigen::Quaterniond(R);
	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();
	odom.pose.pose.orientation.w = q.w();
	odom.pose.pose.position.x = tf(0, 3);
	odom.pose.pose.position.y = tf(1, 3);
	odom.pose.pose.position.z = tf(2, 3);
}

// added by jjiao
double rad2deg(double radians)
{
	return radians * 180.0 / M_PI;
}

double deg2rad(double degrees)
{
	return degrees * M_PI / 180.0;
}

namespace common
{
	/**
	* Inherits from message_filters::SimpleFilter<M>
	* to use protected signalMessage function 
	*/
	template <class M>
	class BagSubscriber : public message_filters::SimpleFilter<M>
	{
	public:
		void newMessage(const boost::shared_ptr<M const> &msg)
		{
			signalMessage(msg);
		}
	};

	template <typename PointT>
	static void publishCloud(const ros::Publisher& publisher,
								const std_msgs::Header& header,
								const typename pcl::PointCloud<PointT>& cloud)
	{
		if (cloud.size()) {
			sensor_msgs::PointCloud2 msg_cloud;
			pcl::toROSMsg(cloud, msg_cloud);
			msg_cloud.header = header;
			publisher.publish(msg_cloud);
		}
	}
}

template <typename T>
void vectorToFile(string filename, vector<T> v_output)
{
    std::ofstream ofs(filename.c_str(), std::ios::app | std::ios::binary);
    if (!ofs.is_open())
    {
        return;
    }	
	for (size_t i = 0; i < v_output.size(); i++)
	{
		ofs << i << std::endl << v_output[i] << std::endl;
	}
}

void matrix_to_vector(const Eigen::Matrix4f &tf, std::vector<float> &p)
{
	p.clear();
	p.resize(7);
	p[0] = tf(0, 3);
	p[1] = tf(1, 3);
	p[2] = tf(2, 3);
	Eigen::Matrix3f R = tf.topLeftCorner(3, 3);
	Eigen::Quaternionf q = Eigen::Quaternionf(R);
	p[3] = q.w();
	p[4] = q.x();
	p[5] = q.y();
	p[6] = q.z();	
}

#endif








