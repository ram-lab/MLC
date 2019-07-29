//
// 1. follow http://wiki.ros.org/rosbag/Cookbook to fullfill the messagefilter from rosbag
// 2. multiple threads: http://www.cppblog.com/ming81/archive/2012/07/18/184028.html
//

#include "utility.h"

#include "pointmatcher_ros/point_cloud.h"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

std::shared_ptr<PM::Transformation> rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

class LidarCloud
{
public:	
	void systemInitialization()
	{}
	
	void setLidarCloud(const DP &cornerCloud, 
					const DP &surfCloud,
					const DP &fullCloud,
					std_msgs::Header header)
	{
		cloud_ = cornerCloud;
		surfCloud_ = surfCloud;
		fullCloud_ = fullCloud; 
		header_ = header;
	}	
	
	void setRaw()
	{
		cloud_raw_ = cloud_;
		surfCloud_raw_ = surfCloud_;
		fullCloud_raw_ = fullCloud_;	
	}
	
	void transformCoordinate()
	{
		PM::TransformationParameters T = PM::TransformationParameters::Identity(4, 4);
		T.topLeftCorner(3, 3) = Eigen::AngleAxisf(2.0944, Eigen::Vector3f(0.5774, 0.5774, 0.5774)).toRotationMatrix();
		cloud_ = rigidTrans->compute(cloud_, T);
		surfCloud_ = rigidTrans->compute(surfCloud_, T);
		fullCloud_ = rigidTrans->compute(fullCloud_, T);
		setRaw();
	}
	
	void transformInitialGuess(const Eigen::Matrix4f &T)
	{
		cloud_ = rigidTrans->compute(cloud_, T);
		surfCloud_ = rigidTrans->compute(surfCloud_, T);
		fullCloud_ = rigidTrans->compute(fullCloud_, T);
		setRaw();
	}
	
	DP cloud_;
    DP surfCloud_;
    DP fullCloud_;	
	DP cloud_raw_;
    DP surfCloud_raw_;
    DP fullCloud_raw_;		
	std_msgs::Header header_;
	
};

void ReadArg(char *argv, std::vector<std::string> &v_str_bag_file, std::vector<Eigen::Matrix4f> &v_tf_ini, std::vector<Eigen::Matrix4f> &v_tf_gt)
{
	std::string fn(argv);
	std::cout << fn << std::endl;
	YAML::Node config = YAML::LoadFile(fn);
	
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
// 	tf_lidar_camera.topLeftCorner(3, 3) = Eigen::AngleAxisf(2.0944, Eigen::Vector3f(0.5774, 0.5774, 0.5774)).toRotationMatrix();	
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
	tf.setIdentity();	
	if (config["tf_l2_l3_ini"])
	{
		for(uint32_t i=0; i<4; i++ )
		{
			for(uint32_t j=0; j<4; j++)
			{
				tf(i,j) = config["tf_l2_l3_ini"][i][j].as<float>();
			}
		}
		v_tf_ini.push_back(tf_lidar_camera.inverse() * tf * tf_lidar_camera);	
	}	
// 	std::cout << "tf_L1-L1: " << std::endl << v_tf_ini[0] << std::endl
// 		<< "tf_L1-L2: " << std::endl << v_tf_ini[1] << std::endl
// 		<< "tf_L1-L3: " << std::endl << v_tf_ini[2] << std::endl;
	std::cout << "tf_INI_L2-L3: " << std::endl << v_tf_ini[3] << std::endl;
		
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
	tf.setIdentity();
	if (config["tf_l2_l3_gt"])
	{
		for(uint32_t i=0; i<4; i++ )
		{
			for(uint32_t j=0; j<4; j++)
			{
				tf(i,j) = config["tf_l2_l3_gt"][i][j].as<float>();
			}
		}
		v_tf_gt.push_back(tf_lidar_camera.inverse() * tf * tf_lidar_camera);	
	}	
// 	std::cout << "tf_GT_L1-L1: " << std::endl << v_tf_gt[0] << std::endl
// 		<< "tf_GT_L1-L2: " << std::endl << v_tf_gt[1] << std::endl
// 		<< "tf_GT_L1-L3: " << std::endl << v_tf_gt[2] << std::endl;
	std::cout << "tf_GT_L2-L3: " << std::endl << v_tf_gt[3] << std::endl;
}

void ReadPointCloudMsg(LidarCloud &lidar_map, rosbag::View &view, std::vector<std::string> topics)
{
// 	common::BagSubscriber<sensor_msgs::PointCloud2> cornerSub, cornerLessSub, flatSub, flatLessSub;
// 	message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> 
// 		sync(cornerSub, cornerLessSub, flatSub, flatLessSub, 25);
// 	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));	
	
	DP cornerCloud, surfCloud, fullCloud;
	BOOST_FOREACH (rosbag::MessageInstance const m, view)
	{	
		if (m.getTopic() == topics[0] || "/" + m.getTopic() == topics[0])
		{
			sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg != nullptr)
			{
				cornerCloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*msg);
			}
		}
		
		if (m.getTopic() == topics[1] || "/" + m.getTopic() == topics[1])
		{
			sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg != nullptr)
			{
				surfCloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*msg);	
			}
		}		
		
		if (m.getTopic() == topics[2] || "/" + m.getTopic() == topics[2])
		{		
			sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg != nullptr)
			{
				fullCloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*msg);		
			}
		}
		
		if (cornerCloud.features.cols() && surfCloud.features.cols() && fullCloud.features.cols()) 
		{
			std_msgs::Header header;
			header.frame_id = "rslidar";
			header.stamp = ros::Time::now();
			lidar_map.setLidarCloud(cornerCloud, surfCloud, fullCloud, header);
			std::cout << "Corner: " << cornerCloud.features.cols() << " Surf: " << surfCloud.features.cols() 
				 << " Full: " << fullCloud.features.cols() << std::endl;
			break;
		}
	}
}

void rotm2eul(const Eigen::Matrix3f &R, Eigen::Vector3f &eul_zyx)
{
	eul_zyx(1) = asin(-R(2,0));
	eul_zyx(0) = asin(R(2,1) / cos(eul_zyx(1)));
	eul_zyx(2) = -atan2(R(1,0), R(0,0));
}

int main(int argc, char** argv)
{
	if(argc < 4)
	{
		std::cerr << "E: no enough argument(s)" << std::endl
				  << "Usage: " << argv[0] << " data_lidar cfg.yaml" << std::endl;
		return 1;
	}
	
	ros::init(argc, argv, "icp");
    ROS_INFO("\033[1;32m---->\033[0m ICP Started.");	
	
	google::InitGoogleLogging(argv[0]);
	
	ros::NodeHandle nh;
	ros::Rate loop_rate(1);
	
	ros::Publisher pubRefCornerCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/corner_cloud", 5);
	ros::Publisher pubRefSurfCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/surf_cloud", 5);
	ros::Publisher pubRefFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/full_cloud", 5);
	
	ros::Publisher pubDataCornerCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/corner_cloud", 5);
	ros::Publisher pubDataSurfCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/surf_cloud", 5);
	ros::Publisher pubDataFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/full_cloud", 5);
	
	ros::Publisher pubDataCornerCloudTrans = nh.advertise<sensor_msgs::PointCloud2>("/data/corner_cloud_trans", 5);
	ros::Publisher pubDataSurfCloudTrans = nh.advertise<sensor_msgs::PointCloud2>("/data/surf_cloud_trans", 5);
	ros::Publisher pubDataFullCloudTrans = nh.advertise<sensor_msgs::PointCloud2>("/data/full_cloud_trans", 5);	

	std::vector<std::string> topics;
	topics.push_back(std::string("/laser_cloud_surround_local_corner"));
	topics.push_back(std::string("/laser_cloud_surround_local_flat"));
	topics.push_back(std::string("/laser_cloud_surround_local"));
// 	topics.push_back(std::string("/laser_cloud_sharp"));
// 	topics.push_back(std::string("/laser_cloud_flat"));
// 	topics.push_back(std::string("/ground_cloud"));	
	
	// read bag_file and initiali tf from yaml
	std::cout << "\033[1;32m<----\033[0m INI_TF" << std::endl;
	std::vector<std::string> v_str_bag_file;
	std::vector<Eigen::Matrix4f> v_tf_ini, v_tf_gt;
	ReadArg(argv[2], v_str_bag_file, v_tf_ini, v_tf_gt);
	
	// read lidar_data from bag_file
	std::cout << "\033[1;32m<----\033[0m SET_LIDAR_DATA" << std::endl;
	std::vector<LidarCloud> v_lidar_pm;
	for (size_t i = 0; i < v_str_bag_file.size(); i++)
	{
		LidarCloud lidar_pm;
		lidar_pm.systemInitialization();
		
		rosbag::Bag bag;
		bag.open(v_str_bag_file[i], rosbag::bagmode::Read);
		rosbag::View view(bag, rosbag::TopicQuery(topics));
		ReadPointCloudMsg(lidar_pm, view, topics);
		
		lidar_pm.transformCoordinate(); // transform pointcloud from camera coordinate to lidar coordinate
// 		lidar_pm.transformInitialGuess(v_tf_ini[i]); // apply the value of initailization
		v_lidar_pm.push_back(lidar_pm);
		
		bag.close();
	}
	
	// TODO: front - tail
	Eigen::Matrix4f iniT, gtT;
	LidarCloud ref_lidar_pm = v_lidar_pm[1];
	LidarCloud data_lidar_pm;
	std::string str_data_lidar(argv[1]);
// 	if (str_data_lidar == "top")
// 	{
// 		data_lidar_pm = v_lidar_pm[0];
// 		iniT = v_tf_ini[0];
// 		gtT = v_tf_gt[0];
// 	} else 
// 	if (str_data_lidar == "front")
// 	{
// 		data_lidar_pm = v_lidar_pm[1];
// 		iniT = v_tf_ini[1];
// 		gtT = v_tf_gt[1];
// 	} else 		
	if (str_data_lidar == "tail")
	{
		iniT = v_tf_ini[3];
		gtT = v_tf_gt[3];
		v_lidar_pm[2].transformInitialGuess(iniT);
		data_lidar_pm = v_lidar_pm[2];
	} 		
	
	// refinement
	std::cout << "\033[1;32m<----\033[0m REFINEMENT" << std::endl;
	float transform_cur[6];
	for (size_t i = 0; i < 6; i++) transform_cur[i] = 0.0;
	PM::TransformationParameters findT_surf = PM::TransformationParameters::Identity(4, 4);
	PM::TransformationParameters findT_corner = PM::TransformationParameters::Identity(4, 4);
	PM::TransformationParameters finalT = PM::TransformationParameters::Identity(4, 4);
	DP cloud_cur;
	
	std::shared_ptr<PM::DataPointsFilter> subSample =
		PM::get().DataPointsFilterRegistrar.create(
			"RandomSamplingDataPointsFilter", 
			{{"prob", "0.25"}}
		);
	ref_lidar_pm.surfCloud_ = subSample->filter(ref_lidar_pm.surfCloud_ );
	data_lidar_pm.surfCloud_ = subSample->filter(data_lidar_pm.surfCloud_ );
	ref_lidar_pm.cloud_ = subSample->filter(ref_lidar_pm.cloud_ );
	data_lidar_pm.cloud_ = subSample->filter(data_lidar_pm.cloud_ );	
	data_lidar_pm.setRaw();
	
	// refinement: surf
	{
		PM::ICP icp_surf;
		std::string fn_config(argv[4]);
		std::ifstream ifs(fn_config);
		if( ifs.good() )
		{
			icp_surf.loadFromYaml(ifs);
			ifs.close();
		}
		else 
		{
			icp_surf.setDefault();
		}
		cloud_cur = data_lidar_pm.surfCloud_;
		for (size_t iter = 0; iter < 1; iter++)
		{
			PM::TransformationParameters T = icp_surf(cloud_cur, ref_lidar_pm.surfCloud_); // icp(data, ref)
			PM::TransformationParameters deltaT = PM::TransformationParameters::Identity(4, 4);
			Eigen::Matrix3f R = T.topLeftCorner(3, 3);
			Eigen::Vector3f eul_zyx; 
			rotm2eul(R, eul_zyx);
			const float tz = T(2, 3);
			deltaT = T;
// 			deltaT.topLeftCorner(3, 3) = (Eigen::AngleAxisf(eul_zyx(1), Eigen::Vector3f::UnitY()) 
// 										* Eigen::AngleAxisf(eul_zyx(2), Eigen::Vector3f::UnitX())).toRotationMatrix();
// 			deltaT(2, 3) = tz;
			cloud_cur = rigidTrans->compute(cloud_cur, deltaT);
			findT_surf *= deltaT;
			
// 			std::cout << std::endl << T << std::endl;;
// 			std::cout << std::endl << eul_zyx.transpose() << std::endl;
			std::cout << "\033[1;32m<----\033[0m ITER: " << iter << " deltaT: " << std::endl << deltaT << std::endl << std::endl;
		}
		std::cout << "\033[1;32m---->\033[0m Surf T :" << std::endl << findT_surf << std::endl << std::endl;	
		
		data_lidar_pm.cloud_ = rigidTrans->compute(data_lidar_pm.cloud_, findT_surf);
		data_lidar_pm.surfCloud_ = rigidTrans->compute(data_lidar_pm.surfCloud_, findT_surf);
		data_lidar_pm.fullCloud_ = rigidTrans->compute(data_lidar_pm.fullCloud_, findT_surf);
	}
	
	// refinement: corner
	{
		PM::ICP icp_corner;
		std::string fn_config(argv[3]);
		std::ifstream ifs(fn_config);
		if( ifs.good() )
		{
			icp_corner.loadFromYaml(ifs);
			ifs.close();
		}
		else
		{
			icp_corner.setDefault();
		}
		cloud_cur = data_lidar_pm.cloud_;
		for (size_t iter = 0; iter < 0; iter++)
		{
			PM::TransformationParameters T = icp_corner(cloud_cur, ref_lidar_pm.cloud_); // icp(data, ref)
			PM::TransformationParameters deltaT = PM::TransformationParameters::Identity(4, 4);
			Eigen::Matrix3f R = T.topLeftCorner(3, 3);
			Eigen::Vector3f eul_zyx;
			rotm2eul(R, eul_zyx);
			const float tx = T(0, 3);
			const float ty = T(1, 3);
			deltaT.topLeftCorner(3, 3) = Eigen::AngleAxisf(eul_zyx(0), Eigen::Vector3f::UnitZ()).toRotationMatrix();
			deltaT(0, 3) = tx;
			deltaT(1, 3) = ty;
			cloud_cur = rigidTrans->compute(cloud_cur, deltaT);
			findT_corner *= deltaT;
			
// 			std::cout << std::endl << T << std::endl;;
// 			std::cout << std::endl << eul_zyx.transpose() << std::endl;
			std::cout << "\033[1;32m<----\033[0m ITER: " << iter << " deltaT: " << std::endl << deltaT << std::endl << std::endl;
		}
		std::cout << "\033[1;32m---->\033[0m Corner T :" << std::endl << findT_corner << std::endl << std::endl;
		
		data_lidar_pm.cloud_ = rigidTrans->compute(data_lidar_pm.cloud_, findT_corner);
		data_lidar_pm.surfCloud_ = rigidTrans->compute(data_lidar_pm.surfCloud_, findT_corner);
		data_lidar_pm.fullCloud_ = rigidTrans->compute(data_lidar_pm.fullCloud_, findT_corner);		
	}

	finalT = iniT * findT_surf * findT_corner;
	std::cout << "\033[1;32m---->\033[0m Final T :" << std::endl << finalT << std::endl << std::endl;	
	std::cout << "\033[1;32m---->\033[0m GT T :" << std::endl << gtT << std::endl;	
	
	// visualization of initial guess
	std::cout << "\033[1;32m<----\033[0m VISUALIZATION" << std::endl;
	while (ros::ok())
	{
		sensor_msgs::PointCloud2 msg;
		std_msgs::Header header = ref_lidar_pm.header_;
		
		pubRefSurfCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_lidar_pm.surfCloud_, header.frame_id, header.stamp)); 
// 		pubRefCornerCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_lidar_pm.cornerCloud_, header.frame_id, header.stamp)); 
// 		pubRefFullCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_lidar_pm.fullCloud_, header.frame_id, header.stamp)); 
		
		header = data_lidar_pm.header_;
		pubDataSurfCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_lidar_pm.surfCloud_raw_, header.frame_id, header.stamp)); 
// 		pubDataCornerCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_lidar_pm.cornerCloud_raw_, header.frame_id, header.stamp)); 
// 		pubDataFullCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_lidar_pm.fullCloud_raw_, header.frame_id, header.stamp)); 
	
		pubDataSurfCloudTrans.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
			(data_lidar_pm.surfCloud_, header.frame_id, header.stamp)); 
// 		pubDataCornerCloudTrans.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
// 			(data_lidar_pm.cornerCloud_, header.frame_id, header.stamp)); 
// 		pubDataFullCloudTrans.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
// 			(data_lidar_pm.fullCloud_, header.frame_id, header.stamp)); 		

		loop_rate.sleep();
	}
}





