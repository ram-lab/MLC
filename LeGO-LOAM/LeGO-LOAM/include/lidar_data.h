#ifndef LIDAR_DATA_H_
#define LIDAR_DATA_H_

#include "utility.h"

class LidarData
{
public:
	void setLidarData(const pcl::PointCloud<PointType> &cPointsSharp, 
					const pcl::PointCloud<PointType> &cPointsLessSharp,
					const pcl::PointCloud<PointType> &sPointsFlat,
					const pcl::PointCloud<PointType> &sPointsLessFlat,
					std_msgs::Header header)
	{
		*cornerPointsSharp = cPointsSharp;
		*cornerPointsLessSharp = cPointsLessSharp;
		*surfPointsFlat = sPointsFlat; 
		*surfPointsLessFlat = sPointsLessFlat; 
		header_ = header;
	}
	
	void systemInitialization()
	{
		cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
		cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
		surfPointsFlat.reset(new pcl::PointCloud<PointType>());
		surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());		
	}

	void transformCoordinate(int sign = 1)
	{
		Eigen::Affine3f tf = Eigen::Affine3f::Identity();
		tf.rotate(Eigen::AngleAxisf(2.0944, Eigen::Vector3f(0.5774, 0.5774, 0.5774)*sign));
		pcl::transformPointCloud(*cornerPointsSharp, *cornerPointsSharp, tf);
		pcl::transformPointCloud(*cornerPointsLessSharp, *cornerPointsLessSharp, tf);
		pcl::transformPointCloud(*surfPointsFlat, *surfPointsFlat, tf);
		pcl::transformPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, tf);
	}
	
	void transformInitialGuess(const Eigen::Matrix4f &tf_ini)
	{
		Eigen::Affine3f tf = Eigen::Affine3f::Identity();
		tf.matrix() = tf_ini;
		pcl::transformPointCloud(*cornerPointsSharp, *cornerPointsSharp, tf);
		pcl::transformPointCloud(*cornerPointsLessSharp, *cornerPointsLessSharp, tf);
		pcl::transformPointCloud(*surfPointsFlat, *surfPointsFlat, tf);
		pcl::transformPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, tf);
	}	
	
	pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
	
	std_msgs::Header header_;
};

class LidarMap
{
public:
	void setLidarMap(const pcl::PointCloud<PointType> &cornerCloud, 
					const pcl::PointCloud<PointType> &surfCloud,
					const pcl::PointCloud<PointType> &fullCloud,
					std_msgs::Header header)
	{
		*cornerCloud_ = cornerCloud;
		*surfCloud_ = surfCloud;
		*fullCloud_ = fullCloud; 
		header_ = header;
	}
	
	void systemInitialization()
	{
		cornerCloud_.reset(new pcl::PointCloud<PointType>());
		surfCloud_.reset(new pcl::PointCloud<PointType>());
		fullCloud_.reset(new pcl::PointCloud<PointType>());
	}

	void transformCoordinate(int sign = 1)
	{
		Eigen::Affine3f tf = Eigen::Affine3f::Identity();
		tf.rotate(Eigen::AngleAxisf(2.0944, Eigen::Vector3f(0.5774, 0.5774, 0.5774)*sign));
		pcl::transformPointCloud(*cornerCloud_, *cornerCloud_, tf);
		pcl::transformPointCloud(*surfCloud_, *surfCloud_, tf);
		pcl::transformPointCloud(*fullCloud_, *fullCloud_, tf);
	}
	
	void transformInitialGuess(const Eigen::Matrix4f &tf_ini)
	{
		Eigen::Affine3f tf = Eigen::Affine3f::Identity();
		tf.matrix() = tf_ini;
		pcl::transformPointCloud(*cornerCloud_, *cornerCloud_, tf);
		pcl::transformPointCloud(*surfCloud_, *surfCloud_, tf);
		pcl::transformPointCloud(*fullCloud_, *fullCloud_, tf);
	}	
	
	pcl::PointCloud<PointType>::Ptr cornerCloud_;
    pcl::PointCloud<PointType>::Ptr surfCloud_;
    pcl::PointCloud<PointType>::Ptr fullCloud_;
	
	std_msgs::Header header_;
};

#endif



