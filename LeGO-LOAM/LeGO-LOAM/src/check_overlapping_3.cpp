//
// 1. follow http://wiki.ros.org/rosbag/Cookbook to fullfill the messagefilter from rosbag
// 2. multiple threads: http://www.cppblog.com/ming81/archive/2012/07/18/184028.html
//

#include "check_overlapping.h"

std::shared_ptr<PM::Transformation> rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

int main(int argc, char** argv)
{
	if(argc < 1)
	{
		std::cerr << "E: no enough argument(s)" << std::endl
				  << "Usage: " << argv[0] << " data_lidar cfg.yaml" << std::endl;
		return 1;
	}
	
	ros::init(argc, argv, "check_overlapping 3");
    ROS_INFO("\033[1;32m---->\033[0m ICP Started.");	
		
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);
	
	ros::Publisher pubRefCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/cloud", 10);
	ros::Publisher pubRefCloudOverlap = nh.advertise<sensor_msgs::PointCloud2>("/ref/cloud_overlap", 10);
	ros::Publisher pubDataCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/cloud", 10);
	ros::Publisher pubDataCloudIniTrans = nh.advertise<sensor_msgs::PointCloud2>("/data/cloud_ini_trans", 10);	
	ros::Publisher pubDataCloudFinalTrans = nh.advertise<sensor_msgs::PointCloud2>("/data/cloud_final_trans", 10);
	
	PM::TransformationParameters iniT = PM::TransformationParameters::Identity(4, 4);		
	PM::TransformationParameters finalT = PM::TransformationParameters::Identity(4, 4);	
	PM::TransformationParameters tf_top_front = PM::TransformationParameters::Identity(4, 4);		
	PM::TransformationParameters tf_top_tail = PM::TransformationParameters::Identity(4, 4);
	PM::TransformationParameters tf = PM::TransformationParameters::Identity(4, 4);
	
	// trajectory_8_shape
// 	tf_top_front << 0.99727, -0.050904,  0.053459,  -0.36009,  0.052233,   0.99835,  -0.02376,0.13834, -0.052161,  0.026487,  0.99829,   -1.2597,  0,   0,  0,   1;
	
	// trajectory_eclipse
	tf_top_front << 0.99636, -0.055485, 0.064747, 0.098638, 0.055005, 0.99844, 0.0091742, 0.34635, -0.065155, -0.0055794, 0.99786, -1.0054, 0, 0, 0, 1.0000;	
	
	tf = tf_top_front;
	std::cout << tf << std::endl;
	
	std::vector<std::vector<DP> > v_lidar_cloud;
	v_lidar_cloud.resize(2);
	
	std::vector<std::string> v_str_bag_file;
// 	v_str_bag_file.push_back("/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_top.bag");
// 	v_str_bag_file.push_back("/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_front.bag");
	v_str_bag_file.push_back("/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/eclipse/odom_top.bag");	
	v_str_bag_file.push_back("/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/eclipse/odom_front.bag");
	std::vector<std::string> topics;
	topics.push_back(std::string("/full_cloud_projected"));
	for (size_t i = 0; i < v_str_bag_file.size(); i++)
	{
		rosbag::Bag bag;
		bag.open(v_str_bag_file[i], rosbag::bagmode::Read);
		rosbag::View view(bag, rosbag::TopicQuery(topics));
		ReadPointCloudMsgDP(v_lidar_cloud[i], view, topics);
		bag.close();
	}	
	
	PM::ICP icp;
	std::string fn_config("/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/config/icp_surf_top_front.yaml");
	std::ifstream ifs(fn_config);
	if( ifs.good() )
	{
		icp.loadFromYaml(ifs);
		ifs.close();
	}
	else 
	{
		icp.setDefault();
	}
	
	std::string filename = "/home/jjiao/Documents/matlab_ws/matlab_multi_lidar_cal/iros_2019/data/refinement_data/top_front_estimated.txt";
    std::ofstream ofs(filename.c_str(), std::ios::out | std::ios::binary);
// 	ofs << "#index px py pz qw qx qy qz omega normal error" << std::endl;
	
	iniT = tf;
	for (size_t i = 0; i < v_lidar_cloud[0].size(); i++)
	{
		std::cout << "i: " << i << std::endl;
		
// 		DP ref_lidar_cloud = DP::load("/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/cloud_top.pcd");
// 		DP data_lidar_cloud = DP::load("/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/cloud_front.pcd");
// 		std::cout << ref_lidar_cloud.features.cols() << " " << data_lidar_cloud.features.cols() << std::endl;	
		DP ref_lidar_cloud = v_lidar_cloud[0][i];
		DP data_lidar_cloud = v_lidar_cloud[1][i];

		DP ref_overlap, data_overlap;	
		DP data_lidar_cloud_ini_trans = rigidTrans->compute(data_lidar_cloud, tf);
		
		// overlap filter
		float omega;
		cloudOverlapEstimationDP(ref_lidar_cloud, data_lidar_cloud_ini_trans, ref_overlap, data_overlap, omega);
		std::cout << "omega: " << omega << std::endl;
		
		ref_lidar_cloud = ref_overlap;
		data_lidar_cloud = rigidTrans->compute(data_overlap, tf.inverse());
		
		// filter
		std::shared_ptr<PM::DataPointsFilter> subsample =
			PM::get().DataPointsFilterRegistrar.create(
				"BoundingBoxDataPointsFilter", PM::Parameters({
				{
					{"xMin", "-3.0"},
					{"xMax", "20"},
					{"yMin", "-20"},
					{"yMax", "20"},
					{"zMin", "-inf"},
					{"zMax", "inf"},
					{"removeInside", "0"}
				}
				}));
		ref_lidar_cloud = subsample->filter(ref_lidar_cloud);
		data_lidar_cloud = subsample->filter(data_lidar_cloud);
		
		subsample =
			PM::get().DataPointsFilterRegistrar.create(
				"BoundingBoxDataPointsFilter", PM::Parameters({
				{
					{"xMin", "0.0"},
					{"xMax", "3.0"},
					{"yMin", "-3"},
					{"yMax", "3"},
					{"zMin", "-inf"},
					{"zMax", "inf"},				
					{"removeInside", "1"}
				}
				}));	
		data_lidar_cloud = subsample->filter(data_lidar_cloud);	
		
		subsample = 
			PM::get().DataPointsFilterRegistrar.create(
				"SamplingSurfaceNormalDataPointsFilter", PM::Parameters({
					{"knn", "20"}
				}));
		DP ref_lidar_normal = subsample->filter(ref_lidar_cloud);
// 		std::cout << ref_lidar_normal.features.cols() << std::endl;

		TP T = TP::Identity(4, 4);
		T = icp(data_lidar_cloud, ref_lidar_cloud, iniT); // icp(data, ref)
		finalT = T;
	// 	finalT(2, 3) = tf_top_front(2,3);
		
		std::cout << "\033[1;32m---->\033[0m tf_ini :" << std::endl << iniT << std::endl;
		std::cout << "\033[1;32m---->\033[0m T_final = T_ini ? :" << std::endl << finalT << std::endl;
// 		iniT = finalT;	

		std::vector<float> pose;
		matrix_to_vector(finalT.block<4,4>(0,0), pose);
		ofs << i << " ";
		for (size_t i = 0; i < pose.size(); i++)
		{
			ofs << pose[i] << " ";
		}	
		ofs << omega << " " << ref_lidar_normal.features.cols() << " ";// << std::endl;	
		
		std::string line;
// 		while (true)
		{
			std_msgs::Header header;
			header.stamp = ros::Time::now();
			header.frame_id = "world";

// 			pubRefCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
// 				(ref_lidar_cloud, header.frame_id, header.stamp)); 
// 			pubDataCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
// 				(data_lidar_cloud, header.frame_id, header.stamp)); 
// 			pubDataCloudIniTrans.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
// 				(rigidTrans->compute(data_lidar_cloud, tf_top_front), header.frame_id, header.stamp));			
// 			pubDataCloudFinalTrans.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
// 				(rigidTrans->compute(data_lidar_cloud, finalT), header.frame_id, header.stamp));
			
			pubRefCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
				(v_lidar_cloud[0][i], header.frame_id, header.stamp)); 
			pubDataCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
				(v_lidar_cloud[1][i], header.frame_id, header.stamp)); 
			pubDataCloudIniTrans.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
				(rigidTrans->compute(v_lidar_cloud[1][i], tf_top_front), header.frame_id, header.stamp));			
			pubDataCloudFinalTrans.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
				(rigidTrans->compute(v_lidar_cloud[1][i], finalT), header.frame_id, header.stamp));			
			
			loop_rate.sleep();
			
// 			if (std::getline(std::cin, line)) 
// 				break;
		}
		std::cout << "Finish ..." << std::endl;

		DP refc = rigidTrans->compute(ref_lidar_cloud, PM::TransformationParameters::Identity(4, 4));
		subsample = 
			PM::get().DataPointsFilterRegistrar.create(
				"SamplingSurfaceNormalDataPointsFilter", PM::Parameters({
					{"knn", "20"}
				}));
		refc = subsample->filter(refc);		
		
		DP datac = rigidTrans->compute(data_lidar_cloud, finalT);
		
// 		PM::Matcher* matcher; 
// 		const int knn = 1; 
// 		PM::Parameters params; 
// 		params["knn"] = PointMatcherSupport::toParam(knn); // other parameters possible matcher = 
// 		PM::get().MatcherRegistrar.create("KDTreeMatcher", params); 
		icp.matcher->init(refc); 
		PM::Matches matches = icp.matcher->findClosests(datac);	
		
		PM::OutlierWeights outlierWeights = icp.outlierFilters.compute(datac, refc, matches); 
		float error = 0;
		error = icp.errorMinimizer->getResidualError(datac, refc, outlierWeights, matches); 		
		std::cout << "\033[1;32m---->\033[0m Error: " << std::endl << error << std::endl;
		ofs << error << std::endl;

		if (!ros::ok())
		{
			break;
		}
	}
}





