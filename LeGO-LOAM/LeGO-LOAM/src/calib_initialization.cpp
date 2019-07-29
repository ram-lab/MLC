//
// 1. follow http://wiki.ros.org/rosbag/Cookbook to fullfill the messagefilter from rosbag
// 2. multiple threads: http://www.cppblog.com/ming81/archive/2012/07/18/184028.html
//

#include "utility.h"
#include "nav_msgs/Odometry.h"

#include "camodocal/EigenUtils.h"
#include "camodocal/calib/PlanarHandEyeCalibration.h"

typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > PoseType;

void ReadArg(char *argv, PoseType &v_tf_exp)
{
	std::string fn(argv);
	YAML::Node config = YAML::LoadFile(fn);

	Eigen::Matrix4d tf;
	tf.setIdentity();
	if (config["tf_l2_l3_exp"])
	{
		for(uint32_t i=0; i<4; i++ )
		{
			for(uint32_t j=0; j<4; j++)
			{
				tf(i,j) = config["tf_l2_l3_exp"][i][j].as<double>();
			}
		}
		v_tf_exp.push_back(tf);	
	}	
// 	tf.setIdentity();
// 	if (config["ext_l1_l3_exp"])
// 	{
// 		for(uint32_t i=0; i<4; i++ )
// 		{
// 			for(uint32_t j=0; j<4; j++)
// 			{
// 				tf(i,j) = config["ext_l1_l3_ini"][i][j].as<double>();
// 			}
// 		}
// 		v_ext_exp.push_back(tf);	
// 	}
// 	std::cout << "tf_L1-L1: " << std::endl << v_tf_ini[0] << std::endl
// 		<< "tf_L1-L2: " << std::endl << v_tf_ini[1] << std::endl
// 		<< "tf_L1-L3: " << std::endl << v_tf_ini[2] << std::endl;
}

void loadBag(const std::string str_bag, const std::vector<std::string> &topics, 
			 PoseType &v_pose)
{
	rosbag::Bag bag;
	bag.open(str_bag, rosbag::bagmode::Read);
	rosbag::View view(bag, rosbag::TopicQuery(topics));	

// 	common::BagSubscriber<sensor_msgs::PointCloud2> cornerSub, cornerLessSub, flatSub, flatLessSub;
// 	message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> 
// 		sync(cornerSub, cornerLessSub, flatSub, flatLessSub, 25);
// 	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));	
	BOOST_FOREACH (rosbag::MessageInstance const m, view)
	{	
		if (m.getTopic() == topics[0] || "/" + m.getTopic() == topics[0])
		{
			nav_msgs::OdometryConstPtr msg = m.instantiate<nav_msgs::Odometry>();
			if (msg != nullptr)
			{
				Eigen::Matrix4d tf;
				odom_to_matrix(*msg, tf);
				v_pose.push_back(tf);
			}
		}
		if (!ros::ok()) return;
	}
	bag.close();
}

int main(int argc, char** argv)
{
	if(argc < 4)
	{
		std::cerr << "E: no enough argument(s)" << std::endl
				  << "Usage: " << argv[0] << "cfg.yaml ref.bag data.bag" << std::endl;
		return 1;
	}
	google::InitGoogleLogging(argv[0]);
	
	ros::init(argc, argv, "calib_initialization");
    ROS_INFO("\033[1;32m---->\033[0m Calib Initialization Started.");
	
	ros::NodeHandle nh;
	ros::Rate loop_rate(1);
	
// 	ros::Publisher pubRefCornerCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/corner_cloud", 1);
// 	ros::Publisher pubRefSurfCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/surf_cloud", 1);
// 	ros::Publisher pubRefFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/full_cloud", 1);
// 	
// 	ros::Publisher pubDataCornerCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/corner_cloud", 1);
// 	ros::Publisher pubDataSurfCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/surf_cloud", 1);
// 	ros::Publisher pubDataFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/full_cloud", 1);

	// read yaml
	PoseType v_tf_exp, v_tf_est;
	std::cout << "\033[1;32m---->\033[0m Load cfg.yaml ..." << std::endl;
	ReadArg(argv[1], v_tf_exp);
	
	// read bag
	std::vector<std::string> topics;
	topics.push_back(std::string("/odom_from_start"));
	std::string ref_bag(argv[2]);
	std::string data_bag(argv[3]);
	
	std::vector<PoseType> v_pose;
	PoseType pose;
	size_t s_pose;
	
	std::cout << "\033[1;32m---->\033[0m Load ref bag ..." << std::endl;
	pose.clear();
	loadBag(ref_bag, topics, pose);
	v_pose.push_back(pose);
	
	std::cout << "\033[1;32m---->\033[0m Load data bag ..." << std::endl;
	pose.clear();
	loadBag(data_bag, topics, pose);
	v_pose.push_back(pose);
	
	s_pose = std::min(v_pose[0].size(), v_pose[1].size());
	std::cout << "\033[1;32m---->\033[0m Number of trajectory: " << s_pose << std::endl;
	
	// calibration
	PoseType H1, H2;
	Eigen::Matrix4d H_12, H_12_exp;
	H_12_exp = v_tf_exp[0];
	for (size_t i = 0; i < s_pose; i++)
	{
		H1.push_back(v_pose[0][i]);
		H2.push_back(v_pose[1][i]);
	}
	
    camodocal::PlanarHandEyeCalibration planar_calib;
    planar_calib.setVerbose(true);
	planar_calib.setRefine(false);
	
	std::cout << "\033[1;32m---->\033[0m Add motions ..." << std::endl;
	planar_calib.addMotions(H1, H2);
	
	std::cout << "\033[1;32m---->\033[0m Calibrate ..." << std::endl;
	planar_calib.calibrate(H_12);
	
	std::cout << "Estimated: " << std::endl;
	std::cout << H_12 << std::endl;
	double roll, pitch, yaw; 
	camodocal::mat2RPY<double>(H_12.block<3,3>(0,0), roll, pitch, yaw);
	std::cout << "r, p, y: " << roll << " " << pitch << " " << yaw << std::endl;	
	
	std::cout << "Expected: " << std::endl;
	std::cout << H_12_exp << std::endl;	

	v_tf_est.push_back(H_12);
}












