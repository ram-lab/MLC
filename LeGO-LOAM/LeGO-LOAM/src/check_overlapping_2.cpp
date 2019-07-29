//
// 1. follow http://wiki.ros.org/rosbag/Cookbook to fullfill the messagefilter from rosbag
// 2. multiple threads: http://www.cppblog.com/ming81/archive/2012/07/18/184028.html
//

#include "check_overlapping.h"


std::shared_ptr<PM::Transformation> rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

int main(int argc, char** argv)
{
	if(argc < 4)
	{
		std::cerr << "E: no enough argument(s)" << std::endl
				  << "Usage: " << argv[0] << " data_lidar cfg.yaml" << std::endl;
		return 1;
	}
	
	ros::init(argc, argv, "check_overlapping");
    ROS_INFO("\033[1;32m---->\033[0m ICP Started.");	
	
	google::InitGoogleLogging(argv[0]);
	
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	
	ros::Publisher pubRefCloud = nh.advertise<sensor_msgs::PointCloud2>("/ref/cloud", 10);
	ros::Publisher pubRefCloudOverlap = nh.advertise<sensor_msgs::PointCloud2>("/ref/cloud_overlap", 10);
	ros::Publisher pubDataCloud = nh.advertise<sensor_msgs::PointCloud2>("/data/cloud", 10);
	ros::Publisher pubDataCloudIniTrans = nh.advertise<sensor_msgs::PointCloud2>("/data/cloud_ini_trans", 10);	
	ros::Publisher pubDataCloudFinalTrans = nh.advertise<sensor_msgs::PointCloud2>("/data/cloud_final_trans", 10);

	std::vector<std::string> topics;
	topics.push_back(std::string("/full_cloud_projected"));
	topics.push_back(std::string("/odom_from_start"));
	
	// read bag_file and initiali tf from yaml
	std::cout << "\033[1;32m<----\033[0m INI_TF" << std::endl;
	std::vector<std::string> v_str_bag_file;
	std::vector<Eigen::Matrix4f> v_tf_ini, v_tf_gt;
	ReadArg(argv[2], v_str_bag_file, v_tf_ini, v_tf_gt);
	
	// read lidar_data from bag_file
	std::cout << "\033[1;32m<----\033[0m SET_LIDAR_DATA" << std::endl;
	std::vector<std::vector<Eigen::Matrix4f> > v_odom;	
	v_odom.resize(2);	
	std::vector<std::vector<LidarCloud> > v_lidar_cloud;
	v_lidar_cloud.resize(2);
	std::vector<std::string> v_frame_id = {"front_rslidar", "tail_rslidar"};
	for (size_t i = 0; i < v_str_bag_file.size(); i++)
	{
		rosbag::Bag bag;
		bag.open(v_str_bag_file[i], rosbag::bagmode::Read);
		rosbag::View view(bag, rosbag::TopicQuery(topics));
		ReadPointCloudMsg(v_lidar_cloud[i], v_odom[i], view, topics, v_frame_id[i]);
		bag.close();
		
		for (size_t j = 0; j < v_lidar_cloud[i].size(); j++)
		{
			Eigen::Matrix4f initial = v_tf_ini[i] * v_odom[i][j];
			v_lidar_cloud[i][j].ini_.block<4, 4>(0, 0) = initial.block<4, 4>(0,0);
			v_lidar_cloud[i][j].odom_ = v_odom[i][j];
		}		
	}
	
	// TODO: front - tail
	std::cout << "\033[1;32m<----\033[0m Local Map Size" << std::endl;
	std::vector<LidarCloud> &v_ref_lidar_cloud = v_lidar_cloud[0];
	std::vector<LidarCloud> &v_data_lidar_cloud = v_lidar_cloud[1];
	
	// check overlapping
	std::cout << "\033[1;32m<----\033[0m CHECK OVERALAPPING" << std::endl;
	int delta_k = 160;
	
	std::string filename = "/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/evaluation/pose_graph_node.txt";
	std::vector<float> v_edge;	
	
	for (int i_ref = 0; i_ref < 100; i_ref++)
	{
		std::cout << "\033[1;32m<----\033[0m i_ref: " << i_ref << std::endl;
		
		PM::TransformationParameters iniT = PM::TransformationParameters::Identity(4, 4);		
		PM::TransformationParameters finalT = PM::TransformationParameters::Identity(4, 4);	
			
		double max_ratio = 0;
		int index_overlap = 0;
		DP max_ref_overlap, max_data_overlap;

		std::vector<double> v_omega;
		for (size_t i = std::max(0, i_ref-delta_k); i < std::min(i_ref+delta_k, int(v_ref_lidar_cloud.size())); i++)
		{	
			PM::Matches matches;		
			LidarCloud ref_lidar_cloud = v_ref_lidar_cloud[i_ref];
			LidarCloud data_lidar_cloud = v_data_lidar_cloud[i];
			DP ref_overlap, data_overlap;
			
			double omega = 0;
			cloudOverlapEstimation(ref_lidar_cloud, data_lidar_cloud, ref_overlap, data_overlap, omega);
			v_omega.push_back(omega);
	// 		std::cout << i << ": " << omega << std::endl;
			if (omega > max_ratio)
			{
				max_ratio = omega;
				index_overlap = i;
				max_ref_overlap = ref_overlap;
				max_data_overlap = data_overlap;
			}		
		}
		
		stringstream ss;
// 		ss << "/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/evaluation/omega_" << i_ref << ".txt";
// 		vectorToFile(ss.str(), v_omega);
// 		ss.str("");
// 		std::cout << index_overlap << std::endl;
		
		int i_start, i_end;
		float ratio = 0.9;
		for (i_start = index_overlap - 1; i_start > std::max(0, i_ref-delta_k); i_start--)
			if (v_omega[i_start] < max_ratio * ratio)
				break;
		i_start++;
		for (i_end = index_overlap + 1; i_end < std::min(i_ref+delta_k, int(v_ref_lidar_cloud.size())); i_end++)
			if (v_omega[i_end] < max_ratio * ratio)
				break;	
		i_end--;

		PM::ICP icp;
		std::string fn_config(argv[3]);
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
		
		LidarCloud &ref_lidar_cloud = v_ref_lidar_cloud[i_ref];
		LidarCloud &data_lidar_cloud = v_data_lidar_cloud[index_overlap];
		
		// visualization of initial guess
		std::cout << "\033[1;32m<----\033[0m VISUALIZATION" << std::endl;
		std::cout << i_start << " ---> " << i_end << std::endl;

// 		matrix_to_vector(v_tf_ini[1], v_edge);
// 		vectorToFile(filename, v_edge);	
// 		
// 		
// 		v_edge.clear();
// 		v_edge.push_back(i_ref);
// 		vectorToFile(filename, v_edge);
		
// 		ss << "/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/evaluation/overlap_" << i_ref << ".bag";
		
		bool b_bag = true;
		if (b_bag)
		{
// 			rosbag::Bag bag;
// 			bag.open(ss.str(), rosbag::bagmode::Write);
			for (size_t j = i_start; j <= i_end; j++)
			{
				LidarCloud &ref_lidar_cloud = v_ref_lidar_cloud[i_ref];
				LidarCloud &data_lidar_cloud = v_data_lidar_cloud[j];
				
				TP T = TP::Identity(4, 4);
				T = icp(data_lidar_cloud.cloud_, ref_lidar_cloud.cloud_, data_lidar_cloud.ini_); // icp(data, ref)
				finalT = T;
				finalT(2, 3) = 0;
				
				std::cout << "\033[1;32m---->\033[0m " << j << " :"<< std::endl;
				std::cout << "\033[1;32m---->\033[0m tf_ini :" << std::endl << v_tf_ini[1] << std::endl;
				std::cout << "\033[1;32m---->\033[0m odom_ini: " << std::endl << data_lidar_cloud.odom_ << std::endl;
				std::cout << "\033[1;32m---->\033[0m T_ini = tf_ini*odom_ini :" << std::endl << data_lidar_cloud.ini_ << std::endl;
				std::cout << "\033[1;32m---->\033[0m T_final = T_ini ? :" << std::endl << finalT << std::endl;
				std::cout << "\033[1;32m---->\033[0m tf_est = T_final*odom_ini^(-1): " << std::endl << finalT * data_lidar_cloud.odom_.inverse() << std::endl;
				
// 				if (j != i_start)
// 				{
// 					std::vector<float> v_edge;
// 					matrix_to_vector(v_data_lidar_cloud[j-1].odom_.inverse() * v_data_lidar_cloud[j].odom_, v_edge);
// 					vectorToFile(filename, v_edge);
// 				}
				
				std_msgs::Header header;
				header.stamp = ros::Time::now();
				header.frame_id = "world";
				
// 				std_msgs::Int32 ind;
// 				ind.data = j;
// 				bag.write("/index", header.stamp, ind);
// 				nav_msgs::Odometry msg;
// 				matrix_to_odom(v_tf_ini[1].cast<double>(), msg);
// 				bag.write("/T_ext_ini", header.stamp, msg);
// 				matrix_to_odom(data_lidar_cloud.odom_.cast<double>(), msg);
// 				bag.write("/T_odom_ini", header.stamp, msg);
// 				matrix_to_odom(finalT.block<4,4>(0,0).cast<double>(), msg);
// 				bag.write("/T_icp", header.stamp, msg);
				
				pubRefCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
					(ref_lidar_cloud.cloud_, header.frame_id, header.stamp)); 
				pubDataCloud.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
					(data_lidar_cloud.cloud_, header.frame_id, header.stamp)); 
				pubDataCloudIniTrans.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
					(rigidTrans->compute(data_lidar_cloud.cloud_, data_lidar_cloud.ini_), header.frame_id, header.stamp));			
				pubDataCloudFinalTrans.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
					(rigidTrans->compute(data_lidar_cloud.cloud_, finalT), header.frame_id, header.stamp));
				
				// publish tf
				Eigen::Matrix4f pose;
				Eigen::Vector3f eul_zyx;
				static tf::TransformBroadcaster br;
				tf::Transform transform;
				tf::Quaternion q;
				
				pose = ref_lidar_cloud.ini_;
				rotm2eul(pose.topLeftCorner(3,3), eul_zyx);
				transform.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
				q.setRPY(eul_zyx(2), eul_zyx(1), eul_zyx(0));
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, header.stamp, header.frame_id, ref_lidar_cloud.header_.frame_id)); // world->front_rslidar
				
				pose = data_lidar_cloud.ini_;
				rotm2eul(pose.topLeftCorner(3,3), eul_zyx);
				transform.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
				q.setRPY(eul_zyx(2), eul_zyx(1), eul_zyx(0));
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, header.stamp, header.frame_id, data_lidar_cloud.header_.frame_id)); // world->tail_rslidar
				
	// 			sleep(1);
				
				loop_rate.sleep();
				
				if (!ros::ok()) 
					return 0;
			}
// 			bag.close();
		}
	}
}





