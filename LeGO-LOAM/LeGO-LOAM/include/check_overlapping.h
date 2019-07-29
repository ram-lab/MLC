#ifndef CHECK_OVERLAPPING_H_
#define CHECK_OVERLAPPING_H_



#include "utility.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher/Parametrizable.h"

#include "std_msgs/Int32.h"

typedef PointMatcher<float> PM;
typedef PointMatcherIO<float> PMIO;
typedef PM::Matrix Matrix;
typedef PM::TransformationParameters TP;
typedef PM::DataPoints DP;
typedef PM::Matches Matches;

#define MAX_K 200
#define KDTREE_MAXD 0.5
#define OVERLAP_KDTREE false
#define PRE_FILTER_RADIUS 25

class LidarCloud
{
public:	
	LidarCloud(): theta_wise_(0), theta_counter_(0), r_(PRE_FILTER_RADIUS)
	{
		pose_.setIdentity();
		ini_ = TP::Identity(4, 4);
	}
	
	void systemInitialization()
	{}
	
	void setLidarCloud(const DP &cloud, std_msgs::Header header)
	{
		cloud_ = cloud;
		header_ = header;
	}	
	
	void setRaw()
	{
		cloud_raw_ = cloud_;
	}
	
	void transformCoordinate()
	{
		std::shared_ptr<PM::Transformation> rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");
		PM::TransformationParameters T = PM::TransformationParameters::Identity(4, 4);
		T.topLeftCorner(3, 3) = Eigen::AngleAxisf(2.0944, Eigen::Vector3f(0.5774, 0.5774, 0.5774)).toRotationMatrix();
		cloud_ = rigidTrans->compute(cloud_, T);
		setRaw();
	}
	
	void transformInitialGuess(const Eigen::Matrix4f &T)
	{
		std::shared_ptr<PM::Transformation> rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");		
		cloud_ = rigidTrans->compute(cloud_, T);
		setRaw();
	}
	
	void angleRange()
	{
// 		for (size_t i = 0; i < cloud_.features.cols(); i++)
// 		{
// 			float x = cloud_.features(0,i);
// 			float y = cloud_.features(1,i);
// 			if (x*x + y*y > r_*r_)
// 			{
// 				cloud_.features.col(i).setZero();
// 				continue;
// 			}
// 			float theta = atan2(y, x);
// 			theta_wise_ = std::max(theta_wise_, theta);
// 			theta_counter_ = std::min(theta_counter_, theta);
// 		}
		theta_wise_ = PI/2;
		theta_counter_ = -PI/2;
	}
	
	void printAngleRange()
	{	
		std::cout << "Theta: " << theta_wise_ << " " << theta_counter_ << std::endl;
	}
	DP cloud_;
	DP cloud_raw_;
	DP cloud_overlap_;
	std_msgs::Header header_;
	
	float theta_wise_, theta_counter_, r_;
	Eigen::Matrix4f odom_, pose_; // the odom in the sensor frame
	TP ini_;
};

void cloudOverlapEstimation(LidarCloud &ref_cloud, LidarCloud &data_cloud, 
							DP &ref_overlap, DP &data_overlap, double &omega)
{
	std::shared_ptr<PM::Transformation> rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");
	DP reference = ref_cloud.cloud_;
	DP reading = rigidTrans->compute(data_cloud.cloud_, data_cloud.ini_);

	std::shared_ptr<PM::DataPointsFilter> subSample =
		PM::get().DataPointsFilterRegistrar.create(
			"RandomSamplingDataPointsFilter", 
			{{"prob", "1"}}
		);

	std::shared_ptr<PM::DataPointsFilter> maxDensity =
		PM::get().DataPointsFilterRegistrar.create(
			"MaxDensityDataPointsFilter"
		);
	
	/*std::shared_ptr<PM::DataPointsFilter> cutInHalf;
	cutInHalf = PM::get().DataPointsFilterRegistrar.create(
		"MinDistDataPointsFilter", PM::Parameters({
			{"dim", "1"},
			{"minDist", "0"}
		}));*/

	std::shared_ptr<PM::DataPointsFilter> computeDensity =
		PM::get().DataPointsFilterRegistrar.create(
			"SurfaceNormalDataPointsFilter",
			{
				{"knn", "20"},
				{"keepDensities", "1"}
			}
		);

// 	reading = subSample->filter(reading);
// 	reading = computeDensity->filter(reading);
// 	reading = maxDensity->filter(reading);
	//reading = cutInHalf->filter(reading);
	const Matrix inliersRead = Matrix::Zero(1, reading.features.cols());
	reading.addDescriptor("inliers", inliersRead);

// 	reference = subSample->filter(reference);
// 	reference = computeDensity->filter(reference);
// 	reference = maxDensity->filter(reference);
	const Matrix inliersRef = Matrix::Zero(1, reference.features.cols());
	reference.addDescriptor("inliers", inliersRef);

	//TODO: reverse self and target
	DP self = reading;
	DP target = reference;
	
	std::vector<DP> v_inlier; // data, ref
	v_inlier.push_back(reading);
	v_inlier.push_back(reference);

	for(int l = 0; l < 2; l++)
	{
		const int selfPtsCount = self.features.cols();
		const int targetPtsCount = target.features.cols();

		// Build kd-tree
		int knn = 10;
		int knnAll = 30;
		std::shared_ptr<PM::Matcher> matcherSelf =
			PM::get().MatcherRegistrar.create(
				"KDTreeMatcher",
				{{"knn", PointMatcherSupport::toParam(knn)}}
			);

		std::shared_ptr<PM::Matcher> matcherTarget =
			PM::get().MatcherRegistrar.create(
				"KDTreeVarDistMatcher",
				{
					{"knn", PointMatcherSupport::toParam(knnAll)},
					{"maxDistField", "maxSearchDist"}
				}
			);

		matcherSelf->init(self);
		matcherTarget->init(target);

		Matches selfMatches(knn, selfPtsCount);
		selfMatches = matcherSelf->findClosests(self);

		const Matrix maxSearchDist = selfMatches.dists.colwise().maxCoeff().cwiseSqrt();
		self.addDescriptor("maxSearchDist", maxSearchDist);

		Matches targetMatches(knnAll, targetPtsCount);
		targetMatches = matcherTarget->findClosests(self);

		BOOST_AUTO(inlierSelf, self.getDescriptorViewByName("inliers"));
		BOOST_AUTO(inlierTarget, target.getDescriptorViewByName("inliers"));
		for(int i = 0; i < selfPtsCount; i++)
		{
			for(int k = 0; k < knnAll; k++)
			{
				if (targetMatches.dists(k, i) != Matches::InvalidDist)
				{
					inlierSelf(0,i) = 1.0;
					inlierTarget(0,targetMatches.ids(k, i)) = 1.0;
				}
			}
		}
		
		for (int i = 0; i < selfPtsCount; i++)
		{
			if (inlierSelf(0, i) != 1.0)
			{
				v_inlier[l].features.col(i).setZero();
			}
		}		
		
		PM::swapDataPoints(self, target);
	}
	const BOOST_AUTO(finalInlierSelf, self.getDescriptorViewByName("inliers"));
	const BOOST_AUTO(finalInlierTarget, target.getDescriptorViewByName("inliers"));	
	const double selfRatio = (finalInlierSelf.array() > 0).count()/(double)finalInlierSelf.cols();
	const double targetRatio = (finalInlierTarget.array() > 0).count()/(double)finalInlierTarget.cols();	
	omega = selfRatio * targetRatio;

// 	std::cout << "Inliers ref: " << v_inlier[0].features.cols() << std::endl;
	ref_overlap = v_inlier[1];
	
// 	std::cout << "Inliers data: " << v_inlier[1].features.cols() << std::endl;
	data_overlap = v_inlier[0];	
	
// 	std::cout << "ref -> data: " << selfRatio << " data -> ref: " << targetRatio << std::endl;
// 	std::cout << "omega: " << omega << " ref_overlap: " << ref_overlap.features.cols()
// 		<< " data_overlap: " << data_overlap.features.cols() << std::endl;	
}

void cloudOverlapEstimationDP(DP ref_cloud, DP data_cloud, 
							DP &ref_overlap, DP &data_overlap, float &omega)
{
	std::shared_ptr<PM::Transformation> rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");
	DP reference = ref_cloud;
	DP reading = data_cloud;

	ref_overlap = ref_cloud;
	data_overlap = data_cloud;

	PM::Parametrizable::Parameters params;
	params["knn"] =  PointMatcherSupport::toParam(1); 
	params["epsilon"] =  PointMatcherSupport::toParam(0);		
	params["maxDist"] = PointMatcherSupport::toParam(10);		
	
	PM::Matches matches_1, matches_2;
	std::shared_ptr<PM::Matcher> matcherHausdorff = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);

	matcherHausdorff->init(reference);
	matches_1 = matcherHausdorff->findClosests(reading);
	
	matcherHausdorff->init(reading);
	matches_2 = matcherHausdorff->findClosests(reference);
	
	int c1 = 0, c2 = 0;
	for (size_t i = 0; i < matches_1.dists.cols(); i++)
	{
		if (matches_1.dists(0, i) == Matches::InvalidDist)
		{
			data_overlap.features(0, i) = 0;
			data_overlap.features(1, i) = 0;
			data_overlap.features(2, i) = 0;
			c1++;
		}
	}
	for (size_t i = 0; i < matches_2.dists.cols(); i++)
	{
		if (matches_2.dists(0, i) == Matches::InvalidDist)
		{
			ref_overlap.features(0, i) = 0;
			ref_overlap.features(1, i) = 0;
			ref_overlap.features(2, i) = 0;			
			c2++;
		}
	}	
	omega = 1.0*(data_cloud.features.cols()-c1)*(ref_cloud.features.cols()-c2)/
			(ref_cloud.features.cols() * data_cloud.features.cols());
}


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
	std::cout << "tf_INI_L2-L3: " << std::endl << v_tf_ini[1] << std::endl;
		
	////////////////////////////////////////////////////////////////////////
	tf.setIdentity();	
	v_tf_gt.push_back(tf_lidar_camera.inverse() * tf * tf_lidar_camera);
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

	std::cout << "tf_GT_L2-L3: " << std::endl << v_tf_gt[1] << std::endl;
}

void ReadPointCloudMsg(std::vector<LidarCloud> &v_lidar_map, std::vector<Eigen::Matrix4f> &v_lidar_odom, 
					   rosbag::View &view, std::vector<std::string> topics, std::string &frame_id)
{
	DP cloud;
	Eigen::Matrix4d odom;
	bool b_cc = false, b_sc = false, b_fc = false, b_odom = false;
	Eigen::Matrix4f tf_lidar_camera = Eigen::Matrix4f::Identity();
	tf_lidar_camera.topLeftCorner(3, 3) = Eigen::AngleAxisf(2.0944, Eigen::Vector3f(0.5774, 0.5774, 0.5774)).toRotationMatrix();
	std_msgs::Header header;
	BOOST_FOREACH (rosbag::MessageInstance const m, view)
	{	
		if (m.getTopic() == topics[0] || "/" + m.getTopic() == topics[0])
		{
			sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg != nullptr)
			{
				cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*msg);
				header = msg->header;
				b_cc = true;
			}
		}

		if (m.getTopic() == topics[1] || "/" + m.getTopic() == topics[1])
		{		
			nav_msgs::Odometry::ConstPtr msg = m.instantiate<nav_msgs::Odometry>();
			if (msg != nullptr)
			{
				odom_to_matrix(*msg, odom);
				b_odom = true;
			}
		}		
		
		if (b_cc & b_odom) 
		{
			header.frame_id = frame_id;
			LidarCloud lidar_map;
			lidar_map.setLidarCloud(cloud, header);
			v_lidar_map.push_back(lidar_map);
			v_lidar_odom.push_back(tf_lidar_camera * odom.cast<float>());
			b_cc = false; 
			b_odom = false;
			if (v_lidar_map.size() > MAX_K)
				return;
		}
		
	}
}

void ReadPointCloudMsgDP(std::vector<DP> &v_lidar_cloud, rosbag::View &view, std::vector<std::string> topics)
{
	DP cloud;
	bool b_cc = false;
	BOOST_FOREACH (rosbag::MessageInstance const m, view)
	{	
		if (m.getTopic() == topics[0] || "/" + m.getTopic() == topics[0])
		{
			sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
			if (msg != nullptr)
			{
				cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*msg);
				b_cc = true;
			}
		}

		if (b_cc) 
		{
			b_cc = false;
			v_lidar_cloud.push_back(cloud);
			if (v_lidar_cloud.size() > 1000)
				return;
		}
		
	}
}

void rotm2eul(const Eigen::Matrix3f &R, Eigen::Vector3f &eul_zyx)
{
// 	eul_zyx(1) = asin(-R(2,0));
// 	eul_zyx(0) = asin(R(2,1) / cos(eul_zyx(1)));
// 	eul_zyx(2) = -atan2(R(1,0), R(0,0));
	eul_zyx = R.eulerAngles(2, 1, 0);
}

// class PoseGraph
// {
// public:
// 	std::vector<int> node;
// 	std::map<int, int> edge;
// 	
// 	
// };


// libpointmatcher -> rosmsg -> pcl
// {
// 		pcl::PointCloud<PointType> ref_cloud;
// 		LidarCloud &ref_lidar_cloud = v_ref_lidar_cloud[0];
// 		std_msgs::Header header = ref_lidar_cloud.header_;
// 		sensor_msgs::PointCloud2 msg = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_lidar_cloud.cloud_, header.frame_id, header.stamp);
// 		std::cout << msg.row_step << std::endl;
// 		pcl::fromROSMsg(msg, ref_cloud);
// 		std::cout << ref_cloud.size() << std::endl;
// }

// TODO :KD tree		
#if OVERLAP_KDTREE
		{
			size_t p1 = ref_lidar_cloud.cloud_.features.cols();
			size_t p2 = data_lidar_cloud.cloud_.features.cols();
			size_t s1 = 0;
			size_t s2 = 0;
			
			matcherHausdorff->init(ref_lidar_cloud.cloud_);
			matches = matcherHausdorff->findClosests(data_lidar_cloud.cloud_);
			for (size_t j = 0; j < matches.dists.cols(); j++)
			{
				if (!isinf(matches.dists(0, j)))
				{
					s2 ++;
				}
			}
	// 		std::cout << matches.dists.cols() << " == " << data_lidar_cloud.cloud_.features.cols() << std::endl;
			
			matcherHausdorff->init(data_lidar_cloud.cloud_);
			matches = matcherHausdorff->findClosests(ref_lidar_cloud.cloud_);		
			for (size_t j = 0; j < matches.dists.cols(); j++)
			{
				if (!isinf(matches.dists(0, j)))
				{
					s1 ++;
				}
			}
	// 		std::cout << matches.dists.cols() << " == " << ref_lidar_cloud.cloud_.features.cols() << std::endl;
	// 		std::cout << i << " s1, p1, s2, p2, s1/p1 * s2/p2: " << s1 << " " << p1 << " " << s2 << " " << p2 << " " << (1.0*s1*s2)/(p1*p2) << std::endl;
			
			if ((1.0*s1*s2)/(p1*p2) > max_ratio)
			{
				max_ratio = (1.0*s1*s2)/(p1*p2);
				index_overlap = i;
				matches_overlap = matches;
			}
			
	// 		float maxDist1 = matches.getDistsQuantile(1.0);
	// 		float maxDistRobust1 = matches.getDistsQuantile(0.85);	
	// 		std::cout << "Matching: " << matches.dists.rows() << " " << matches.dists.cols() << std::endl;
	// 		std::cout << matches.dists << std::endl;
	// 		
	// 		matcherHausdorff->init(data_lidar_cloud.cloud_);
	// 		matches = matcherHausdorff->findClosests(ref_lidar_cloud.cloud_);
	// 		float maxDist2 = matches.getDistsQuantile(1.0);
	// 		float maxDistRobust2 = matches.getDistsQuantile(0.85);
	// 		std::cout << "Matching: " << matches.dists.rows() << " " << matches.dists.cols() << std::endl;
	// 
	// 		float haussdorffDist = std::max(maxDist1, maxDist2);
	// 		float haussdorffQuantileDist = std::max(maxDistRobust1, maxDistRobust2);		
	// 		
	// 		std::cout << "Ref, Data, Matches, Maxdist: " << std::endl;
	// 		std::cout << ref_lidar_cloud.cloud_.features.rows() << " " << ref_lidar_cloud.cloud_.features.cols() << std::endl;
	// 		std::cout << data_lidar_cloud.cloud_.features.rows() << " " << data_lidar_cloud.cloud_.features.cols() << std::endl;
	// 		std::cout << "Haussdorff distance: " << std::sqrt(haussdorffDist) << " m" << std::endl;
	// 		std::cout << "Haussdorff quantile distance: " << std::sqrt(haussdorffQuantileDist) <<  " m" << std::endl;		
		}
#endif
	
#endif