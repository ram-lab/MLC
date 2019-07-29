//
// 1. follow http://wiki.ros.org/rosbag/Cookbook to fullfill the messagefilter from rosbag
// 2. multiple threads: http://www.cppblog.com/ming81/archive/2012/07/18/184028.html
//

#include "lidar_data.h"
#include "calib_refinement.h"
#include "utility.h"

bool b_visualization_raw = false;
bool b_visualization_ini = false;

std::vector<LidarData> lidarDataSet;

void ReadArg(char *argv, std::vector<std::string> &v_str_bag_file, std::vector<Eigen::Matrix4f> &v_tf_ini, std::vector<Eigen::Matrix4f> &v_tf_gt)
{
	std::string fn(argv);
	YAML::Node config = YAML::LoadFile(fn);
	
	////////////////////////////////////////////////////////////////////////
	if ((config["visualization_ini"]) && (config["visualization_ini"][0].as<int>() > 0))
	{
		b_visualization_ini = true;
	}
	
	if (!config["bag_file"] || !config["bag_file"].IsSequence())
	{
		std::cerr << "E: read yaml but no node(pointclouds)" << std::endl;
		return;
	} else
	{
		auto bags = config["bag_file"];
		for (size_t i = 0; i < bags.size(); i++)
		{
			v_str_bag_file.push_back(bags[i].as<std::string>());
		}
	}
	
	Eigen::Matrix4f tf_lidar_camera = Eigen::Matrix4f::Identity();
	tf_lidar_camera.topLeftCorner(3, 3) = Eigen::AngleAxisf(2.0944, Eigen::Vector3f(0.5774, 0.5774, 0.5774)).toRotationMatrix();	
	Eigen::Matrix4f tf;

	////////////////////////////////////////////////////////////////////////
	tf.setIdentity();
	v_tf_ini.push_back(tf_lidar_camera.inverse() * tf * tf_lidar_camera);
	if (config["tf_l1_l2_ini"])
	{
		for(uint32_t i=0; i<4; i++ )
		{
			for(uint32_t j=0; j<4; j++)
			{
				tf(i,j) = config["tf_l1_l2_ini"][i][j].as<float>();
			}
		}
		v_tf_ini.push_back(tf_lidar_camera.inverse() * tf * tf_lidar_camera);	
	}	
	tf.setIdentity();
	if (config["tf_l1_l3_ini"])
	{
		for(uint32_t i=0; i<4; i++ )
		{
			for(uint32_t j=0; j<4; j++)
			{
				tf(i,j) = config["tf_l1_l3_ini"][i][j].as<float>();
			}
		}
		v_tf_ini.push_back(tf_lidar_camera.inverse() * tf * tf_lidar_camera);	
	}
	std::cout << "tf_L1-L1: " << std::endl << v_tf_ini[0] << std::endl
		<< "tf_L1-L2: " << std::endl << v_tf_ini[1] << std::endl
		<< "tf_L1-L3: " << std::endl << v_tf_ini[2] << std::endl;
		
		
	////////////////////////////////////////////////////////////////////////
	tf.setIdentity();	
	v_tf_gt.push_back(tf_lidar_camera.inverse() * tf * tf_lidar_camera);
	if (config["tf_l1_l2_gt"])
	{
		for(uint32_t i=0; i<4; i++ )
		{
			for(uint32_t j=0; j<4; j++)
			{
				tf(i,j) = config["tf_l1_l2_gt"][i][j].as<float>();
			}
		}
		v_tf_gt.push_back(tf_lidar_camera.inverse() * tf * tf_lidar_camera);	
	}	
	tf.setIdentity();
	if (config["tf_l1_l3_gt"])
	{
		for(uint32_t i=0; i<4; i++ )
		{
			for(uint32_t j=0; j<4; j++)
			{
				tf(i,j) = config["tf_l1_l3_gt"][i][j].as<float>();
			}
		}
		v_tf_gt.push_back(tf_lidar_camera.inverse() * tf * tf_lidar_camera);	
	}
	std::cout << "tf_GT_L1-L1: " << std::endl << v_tf_gt[0] << std::endl
		<< "tf_GT_L1-L2: " << std::endl << v_tf_gt[1] << std::endl
		<< "tf_GT_L1-L3: " << std::endl << v_tf_gt[2] << std::endl;
}

// TODO:
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg_corner, 
              const sensor_msgs::PointCloud2::ConstPtr &msg_cornerLess, 
              const sensor_msgs::PointCloud2::ConstPtr &msg_flat,
              const sensor_msgs::PointCloud2::ConstPtr &msg_flatLess)
{
// 	LidarData ld;
// 	ld.setLidarData(msg_corner, msg_cornerLess, msg_flat, msg_flatLess);
// 	lidarDataSet.push_back(ld);
}

void ReadPointCloudMsg(LidarData &lidar_data, rosbag::View &view, std::vector<std::string> topics)
{
// 	common::BagSubscriber<sensor_msgs::PointCloud2> cornerSub, cornerLessSub, flatSub, flatLessSub;
// 	message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> 
// 		sync(cornerSub, cornerLessSub, flatSub, flatLessSub, 25);
// 	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));	
	
	pcl::PointCloud<PointType> cPointsSharp, cPointsLessSharp, sPointsFlat, sPointsLessFlat;
	BOOST_FOREACH (rosbag::MessageInstance const m, view)
	{	
		if (m.getTopic() == topics[0] || "/" + m.getTopic() == topics[0])
		{
			sensor_msgs::PointCloud2ConstPtr msg_sharp = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg_sharp != nullptr)
			{
				pcl::fromROSMsg(*msg_sharp, cPointsSharp);
			}
		}
		
		if (m.getTopic() == topics[1] || "/" + m.getTopic() == topics[1])
		{
			sensor_msgs::PointCloud2ConstPtr msg_less_sharp = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg_less_sharp != nullptr)
			{
				pcl::fromROSMsg(*msg_less_sharp, cPointsLessSharp);			
			}
		}		
		
		if (m.getTopic() == topics[2] || "/" + m.getTopic() == topics[2])
		{		
			sensor_msgs::PointCloud2ConstPtr msg_flat = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg_flat != nullptr)
			{
				pcl::fromROSMsg(*msg_flat, sPointsFlat);				
			}
		}
		
		if (m.getTopic() == topics[3] || "/" + m.getTopic() == topics[3])
		{				
			sensor_msgs::PointCloud2ConstPtr msg_less_flat = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg_less_flat != nullptr)
			{
				pcl::fromROSMsg(*msg_less_flat, sPointsLessFlat);
			}
		}
		
		if (cPointsSharp.size() && cPointsLessSharp.size() && sPointsFlat.size() && sPointsLessFlat.size()) 
		{
			std_msgs::Header header;
			header.frame_id = "rslidar";
			header.stamp = ros::Time::now();
			lidar_data.setLidarData(cPointsSharp, cPointsLessSharp, sPointsFlat, sPointsLessFlat, header);
			std::cout << "Sharp: " << cPointsSharp.size() << " Less Sharp: " << cPointsLessSharp.size() 
				 << " Flat: " << sPointsFlat.size() << " Less Flat: " << sPointsLessFlat.size() << std::endl;
			break;
		}
	}
}

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		std::cerr << "E: no enough argument(s)" << std::endl
				  << "Usage: " << argv[0] << " cfg.yaml" << std::endl;
		return 1;
	}
	
	ros::init(argc, argv, "calib_refinement");
    ROS_INFO("\033[1;32m---->\033[0m Calib Refinement Started.");	
	
	google::InitGoogleLogging(argv[0]);
	
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	ros::Publisher pubRefCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/ref/laser_cloud_less_sharp", 1);
	ros::Publisher pubDataCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/data/laser_cloud_less_sharp", 1);	
	ros::Publisher pubDataCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/data/laser_cloud_sharp", 1);		
	
	ros::Publisher pubRefSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/ref/laser_cloud_less_flat", 1);
	ros::Publisher pubDataSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/data/laser_cloud_less_flat", 1);
	ros::Publisher pubDataSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/data/laser_cloud_flat", 1);
	
	std::vector<std::string> topics;
	topics.push_back(std::string("/laser_cloud_sharp"));
	topics.push_back(std::string("/laser_cloud_less_sharp"));
	topics.push_back(std::string("/laser_cloud_flat"));
	topics.push_back(std::string("/laser_cloud_less_flat"));
	
	// read bag_file and initiali tf from yaml
	std::cout << "\033[1;32m<----\033[0m INI_TF" << std::endl;
	std::vector<std::string> v_str_bag_file;
	std::vector<Eigen::Matrix4f> v_tf_ini, v_tf_gt;
	ReadArg(argv[1], v_str_bag_file, v_tf_ini, v_tf_gt);
	
	// read lidar_data from bag_file
	std::cout << "\033[1;32m<----\033[0m SET_LIDAR_DATA" << std::endl;
	std::vector<LidarData> v_lidar_data;
	for (size_t i = 0; i < v_str_bag_file.size(); i++)
	{
		LidarData lidar_data;
		lidar_data.systemInitialization();
		
		rosbag::Bag bag;
		bag.open(v_str_bag_file[i], rosbag::bagmode::Read);
		rosbag::View view(bag, rosbag::TopicQuery(topics));
		ReadPointCloudMsg(lidar_data, view, topics);
		
// 		lidar_data.transformCoordinate(); // transform pointcloud from camera coordinate to lidar coordinate
		lidar_data.transformInitialGuess(v_tf_ini[i]); // apply the value of initailization
		v_lidar_data.push_back(lidar_data);
		
		bag.close();
	}
	
	// refinement
	std::cout << "\033[1;32m<----\033[0m REFINEMENT" << std::endl;
	CalibRefinement cr(nh);
	cr.systemInitialization();
	cr.setData(v_lidar_data[0], v_lidar_data[1], v_tf_ini[1]); 
	cr.runCalibRefinement();
	
	// visualization of initial guess
	std::cout << "\033[1;32m<----\033[0m VISUALIZATION" << std::endl;
	while (ros::ok())
	{
		if (b_visualization_ini)
		{
			common::publishCloud(pubRefCornerPointsLessSharp, v_lidar_data[0].header_, *v_lidar_data[0].cornerPointsLessSharp);
			common::publishCloud(pubDataCornerPointsLessSharp, v_lidar_data[1].header_, *v_lidar_data[1].cornerPointsLessSharp);
			common::publishCloud(pubDataCornerPointsSharp, v_lidar_data[1].header_, *v_lidar_data[1].cornerPointsSharp);
			
			common::publishCloud(pubRefSurfPointsLessFlat, v_lidar_data[0].header_, *v_lidar_data[0].surfPointsLessFlat);
			common::publishCloud(pubDataSurfPointsLessFlat, v_lidar_data[1].header_, *v_lidar_data[1].surfPointsLessFlat);
			common::publishCloud(pubDataSurfPointsFlat, v_lidar_data[1].header_, *v_lidar_data[1].surfPointsFlat);
		}
		cr.publishCloud();
		loop_rate.sleep();
	}
}





