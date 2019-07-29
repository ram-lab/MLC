//
// 1. follow http://wiki.ros.org/rosbag/Cookbook to fullfill the messagefilter from rosbag
// 2. multiple threads: http://www.cppblog.com/ming81/archive/2012/07/18/184028.html
//

#include "utility.h"

#include "pointmatcher_ros/point_cloud.h"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

std::shared_ptr<PM::Transformation> rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

void rotm2eul(const Eigen::Matrix3f &R, Eigen::Vector3f &eul_zyx)
{
	eul_zyx(1) = asin(-R(2,0));
	eul_zyx(0) = asin(R(2,1) / cos(eul_zyx(1)));
	eul_zyx(2) = -atan2(R(1,0), R(0,0));
}

void ReadArg(char *argv, Eigen::Matrix4f &iniT, Eigen::Matrix4f &gtT)
{
	std::string fn(argv);
	std::cout << fn << std::endl;
	YAML::Node config = YAML::LoadFile(fn);

// 	Eigen::Matrix4f tf_lidar_camera = Eigen::Matrix4f::Identity();
// 	tf_lidar_camera.topLeftCorner(3, 3) = Eigen::AngleAxisf(2.0944, Eigen::Vector3f(0.5774, 0.5774, 0.5774)).toRotationMatrix();	
	Eigen::Matrix4f tf;

	////////////////////////////////////////////////////////////////////////
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
		iniT = tf;
	}	
	std::cout << "tf_INI_L2-L3: " << std::endl << iniT << std::endl;
		
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
		gtT = tf;
	}	

	std::cout << "tf_GT_L2-L3: " << std::endl << gtT << std::endl;
}

int main(int argc, char** argv)
{
	if(argc < 5)
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
	
	ros::Publisher pubRefCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/map", 5);
	ros::Publisher pubDataCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/map", 5);
	ros::Publisher pubDataCloudTrans = nh.advertise<sensor_msgs::PointCloud2>("/data/map_trans", 5);
	
	Eigen::Matrix4f iniT, gtT;
	ReadArg(argv[2], iniT, gtT);
	
	// refinement
	std::cout << "\033[1;32m<----\033[0m REFINEMENT" << std::endl;
	float transform_cur[6];
	for (size_t i = 0; i < 6; i++) transform_cur[i] = 0.0;
	PM::TransformationParameters initialT = PM::TransformationParameters::Identity(4, 4);
	PM::TransformationParameters findT_surf = PM::TransformationParameters::Identity(4, 4);
	PM::TransformationParameters findT_corner = PM::TransformationParameters::Identity(4, 4);
	PM::TransformationParameters finalT = PM::TransformationParameters::Identity(4, 4);
	
	initialT.topLeftCorner(3,3) = iniT.topLeftCorner(3,3);
	initialT.topRightCorner(3,1) = iniT.topRightCorner(3,1);
	
// 	std::shared_ptr<PM::DataPointsFilter> subSample =
// 		PM::get().DataPointsFilterRegistrar.create(
// 			"RandomSamplingDataPointsFilter", 
// 			{{"prob", "0.25"}}
// 		);
// 	ref_lidar_pm.surfCloud_ = subSample->filter(ref_lidar_pm.surfCloud_ );
// 	data_lidar_pm.surfCloud_ = subSample->filter(data_lidar_pm.surfCloud_ );
// 	ref_lidar_pm.cornerCloud_ = subSample->filter(ref_lidar_pm.cornerCloud_ );
// 	data_lidar_pm.cornerCloud_ = subSample->filter(data_lidar_pm.cornerCloud_ );	
// 	data_lidar_pm.setRaw();
	
	std::string str_ref(argv[4]);
	std::string str_data(argv[5]);
	
	DP ref_lidar = DP::load(str_ref);
	DP data_lidar = DP::load(str_data);	
	DP data_lidar_trans;
	data_lidar_trans = rigidTrans->compute(data_lidar, initialT);
	data_lidar = rigidTrans->compute(data_lidar, initialT);
	
	// refinement: surf
	{
		PM::ICP icp_surf;
		std::string fn_config(argv[3]);
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
		for (size_t iter = 0; iter < 1; iter++)
		{
			PM::TransformationParameters T = icp_surf(data_lidar_trans, ref_lidar); // icp(data, ref)
			Eigen::Matrix3f R = T.topLeftCorner(3, 3);
			Eigen::Vector3f eul_zyx; 
			rotm2eul(R, eul_zyx);
			const float tz = T(2, 3);
// 			deltaT.topLeftCorner(3, 3) = (Eigen::AngleAxisf(eul_zyx(1), Eigen::Vector3f::UnitY()) 
// 										* Eigen::AngleAxisf(eul_zyx(2), Eigen::Vector3f::UnitX())).toRotationMatrix();
// 			deltaT(2, 3) = tz;
			data_lidar_trans = rigidTrans->compute(data_lidar_trans, T);
			std::cout << "\033[1;32m---->\033[0m Surf T :" << std::endl << T << std::endl << std::endl;	
			findT_surf *= T;
		}
	}

	finalT = initialT * findT_surf * findT_corner;
	std::cout << "\033[1;32m---->\033[0m Final T :" << std::endl << finalT << std::endl << std::endl;	
	
	// visualization of initial guess
	std::cout << "\033[1;32m<----\033[0m VISUALIZATION" << std::endl;
	while (ros::ok())
	{
		sensor_msgs::PointCloud2 msg;
		std_msgs::Header header;
		header.frame_id = "map";
		header.stamp = ros::Time::now();
		
		pubRefCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_lidar, header.frame_id, header.stamp)); 
		pubDataCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_lidar, header.frame_id, header.stamp)); 
		pubDataCloudTrans.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
			(data_lidar_trans, header.frame_id, header.stamp)); 

		loop_rate.sleep();
	}
}





