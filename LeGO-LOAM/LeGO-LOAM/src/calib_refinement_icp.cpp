//
// 1. follow http://wiki.ros.org/rosbag/Cookbook to fullfill the messagefilter from rosbag
// 2. multiple threads: http://www.cppblog.com/ming81/archive/2012/07/18/184028.html
//

#include "lidar_data.h"
#include "calib_refinement_icp.h"
#include "utility.h"

#include "pointmatcher/PointMatcher.h"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

bool b_visualization_raw = false;
bool b_visualization_ini = false;
int data_lidar = 2;

std::vector<LidarData> lidarDataSet;

void ReadArg(char *argv, std::vector<std::string> &v_str_bag_file, std::vector<Eigen::Matrix4f> &v_tf_ini, std::vector<Eigen::Matrix4f> &v_tf_gt)
{
	std::string fn(argv);
	std::cout << fn << std::endl;
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
// 	ld.setLidarMap(msg_corner, msg_cornerLess, msg_flat, msg_flat_less);
// 	lidarDataSet.push_back(ld);
}

void ReadPointCloudMsg(LidarMap &lidar_map, rosbag::View &view, std::vector<std::string> topics)
{
// 	common::BagSubscriber<sensor_msgs::PointCloud2> cornerSub, cornerLessSub, flatSub, flatLessSub;
// 	message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> 
// 		sync(cornerSub, cornerLessSub, flatSub, flatLessSub, 25);
// 	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));	
	
	pcl::PointCloud<PointType> cornerCloud, SurfCloud, fullCloud;
	BOOST_FOREACH (rosbag::MessageInstance const m, view)
	{	
		if (m.getTopic() == topics[0] || "/" + m.getTopic() == topics[0])
		{
			sensor_msgs::PointCloud2ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg != nullptr)
			{
				pcl::fromROSMsg(*msg, cornerCloud);
			}
		}
		
		if (m.getTopic() == topics[1] || "/" + m.getTopic() == topics[1])
		{
			sensor_msgs::PointCloud2ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg != nullptr)
			{
				pcl::fromROSMsg(*msg, SurfCloud);			
			}
		}		
		
		if (m.getTopic() == topics[2] || "/" + m.getTopic() == topics[2])
		{		
			sensor_msgs::PointCloud2ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg != nullptr)
			{
				pcl::fromROSMsg(*msg, fullCloud);				
			}
		}
		
		if (cornerCloud.size() && SurfCloud.size() && fullCloud.size()) 
		{
			std_msgs::Header header;
			header.frame_id = "rslidar";
			header.stamp = ros::Time::now();
			lidar_map.setLidarMap(cornerCloud, SurfCloud, fullCloud, header);
			std::cout << "Corner: " << cornerCloud.size() << " Surf: " << SurfCloud.size() 
				 << " Full: " << fullCloud.size() << std::endl;
			break;
		}
	}
}

int main(int argc, char** argv)
{
	if(argc < 3)
	{
		std::cerr << "E: no enough argument(s)" << std::endl
				  << "Usage: " << argv[0] << " data_lidar cfg.yaml" << std::endl;
		return 1;
	}
	
	ros::init(argc, argv, "calib_refinement_icp");
    ROS_INFO("\033[1;32m---->\033[0m Calib Refinement Started.");	
	
	google::InitGoogleLogging(argv[0]);
	
	ros::NodeHandle nh;
	ros::Rate loop_rate(1);
	
	ros::Publisher pubRefCornerCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/corner_cloud", 1);
	ros::Publisher pubRefSurfCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/surf_cloud", 1);
	ros::Publisher pubRefFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/full_cloud", 1);
	
	ros::Publisher pubDataCornerCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/corner_cloud", 1);
	ros::Publisher pubDataSurfCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/surf_cloud", 1);
	ros::Publisher pubDataFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/full_cloud", 1);
	
	std::vector<std::string> topics;
	topics.push_back(std::string("/laser_cloud_surround_local_corner"));
	topics.push_back(std::string("/laser_cloud_surround_local_flat"));
	topics.push_back(std::string("/laser_cloud_surround_local"));
	
	// read bag_file and initiali tf from yaml
	std::cout << "\033[1;32m<----\033[0m INI_TF" << std::endl;
	std::vector<std::string> v_str_bag_file;
	std::vector<Eigen::Matrix4f> v_tf_ini, v_tf_gt;
	ReadArg(argv[2], v_str_bag_file, v_tf_ini, v_tf_gt);
	
	// read lidar_data from bag_file
	std::cout << "\033[1;32m<----\033[0m SET_LIDAR_DATA" << std::endl;
	std::vector<LidarMap> v_lidar_map;
	for (size_t i = 0; i < v_str_bag_file.size(); i++)
	{
		LidarMap lidar_map;
		lidar_map.systemInitialization();
		
		rosbag::Bag bag;
		bag.open(v_str_bag_file[i], rosbag::bagmode::Read);
		rosbag::View view(bag, rosbag::TopicQuery(topics));
		ReadPointCloudMsg(lidar_map, view, topics);
		
		lidar_map.transformCoordinate(); // transform pointcloud from camera coordinate to lidar coordinate
		lidar_map.transformInitialGuess(v_tf_ini[i]); // apply the value of initailization
		v_lidar_map.push_back(lidar_map);
		
		bag.close();
	}
	
	LidarMap ref_lidar_map = v_lidar_map[0];
	LidarMap data_lidar_map;
	std::string str_data_lidar(argv[1]);
	if (str_data_lidar == "front")
	{
		data_lidar_map = v_lidar_map[1];
	} else 
	if (str_data_lidar == "tail")
	{
		data_lidar_map = v_lidar_map[2];
	} 		
	
	// refinement
// 	std::cout << "\033[1;32m<----\033[0m REFINEMENT" << std::endl;
// 	CalibRefinementICP cri(nh);
// 	cri.systemInitialization();
// 	cri.setData(ref_lidar_map, data_lidar_map, v_tf_ini[1]); 
// 	cri.runCalibRefinement();

	// visualization of initial guess
	std::cout << "\033[1;32m<----\033[0m VISUALIZATION" << std::endl;
	while (ros::ok())
	{
		if (b_visualization_ini)
		{
			common::publishCloud(pubRefCornerCloud, ref_lidar_map.header_, *ref_lidar_map.cornerCloud_);
			common::publishCloud(pubRefSurfCloud, ref_lidar_map.header_, *ref_lidar_map.surfCloud_);
			common::publishCloud(pubRefFullCloud, ref_lidar_map.header_, *ref_lidar_map.fullCloud_);
			
			common::publishCloud(pubDataCornerCloud, data_lidar_map.header_, *data_lidar_map.cornerCloud_);
			common::publishCloud(pubDataSurfCloud, data_lidar_map.header_, *data_lidar_map.surfCloud_);
			common::publishCloud(pubDataFullCloud, data_lidar_map.header_, *data_lidar_map.fullCloud_);
		}
// 		cr.publishCloud();
		loop_rate.sleep();
	}
}





