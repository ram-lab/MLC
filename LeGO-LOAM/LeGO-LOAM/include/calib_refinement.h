#ifndef CALIB_REFINEMENT_H_
#define CALIB_REFINEMENT_H_

#include "visualizer/Visualizer.h"
#include "lidar_data.h"
#include "utility.h"

class CalibRefinement
{
public:
	CalibRefinement(ros::NodeHandle &nh) : nh_(nh)
	{}
	
	void systemInitialization()
	{
		pubDataCornerPointsLessSharpTransformed = nh_.advertise<sensor_msgs::PointCloud2>("/data/laser_cloud_less_sharp_transformed", 1);
		pubDataSurfPointsLessFlatTransformed = nh_.advertise<sensor_msgs::PointCloud2>("/data/laser_cloud_less_flat_transformed", 1);
		
		downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
		
		refLidarData.systemInitialization();
		dataLidarData.systemInitialization();
		
		laserCloudOri.reset(new pcl::PointCloud<PointType>());
		coeffSel.reset(new pcl::PointCloud<PointType>());
	
		kdtreeCornerRef.reset(new pcl::KdTreeFLANN<PointType>());
		kdtreeSurfRef.reset(new pcl::KdTreeFLANN<PointType>());
		
		CornerPointsCorr.resize(2);
		surfPointsCorr.resize(2);
		for (size_t i = 0; i < CornerPointsCorr.size(); i++)
		{
			CornerPointsCorr[i].reset(new pcl::PointCloud<PointType>());
			surfPointsCorr[i].reset(new pcl::PointCloud<PointType>());
		}
	
// 		laserOdometry.header.frame_id = "/camera_init";
// 		laserOdometry.child_frame_id = "/laser_odom";
// 	
// 		laserOdometryTrans.frame_id_ = "/camera_init";
// 		laserOdometryTrans.child_frame_id_ = "/laser_odom";
		
		isDegenerate = false;
		matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
		
		ini_tf.setIdentity();
		estimated_tf.setIdentity();
		refined_tf.setIdentity();
	
// 		frameCount = skipFrameNum;
	}

	void setData(LidarData &ref, LidarData &data, const Eigen::Matrix4f &tf)
	{
		refLidarData.setLidarData(*ref.cornerPointsSharp, *ref.cornerPointsLessSharp, *ref.surfPointsFlat, *ref.surfPointsLessFlat, ref.header_);
		dataLidarData.setLidarData(*data.cornerPointsSharp, *data.cornerPointsLessSharp, *data.surfPointsFlat, *data.surfPointsLessFlat, data.header_);
		ini_tf.matrix() = tf;
	}

	void runCalibRefinement()
	{
		refinementInitialization();
		updateInitialGuess();
		updateTransformation();
// 		printResult();
	}

	void refinementInitialization()
	{
		kdtreeCornerRef->setInputCloud(refLidarData.cornerPointsLessSharp);
		kdtreeSurfRef->setInputCloud(refLidarData.surfPointsLessFlat);
		numCornerRef = refLidarData.cornerPointsLessSharp->size();
		numSurfRef = refLidarData.surfPointsLessFlat->size();
	}
	
	void updateInitialGuess()
	{
		transformCur[0] = 0.0;
		transformCur[1] = 0.0;
		transformCur[2] = 0.0;
		transformCur[3] = 0.0;
		transformCur[4] = 0.0;
		transformCur[5] = 0.0;
	}

	// TODO: find correspondences of edge and planar features, and calculate R|t
	void updateTransformation()
	{
		std::cout << "Number of ref corner and surf: " << numCornerRef << " " << numSurfRef << std::endl;
		std::cout << "Number of data corner and surf: " << dataLidarData.cornerPointsSharp->size() 
			<< " " << dataLidarData.surfPointsFlat->size() << std::endl;
		
		if (numCornerRef < 10 || numSurfRef < 100)
			return;
	
		PlaneNormalVisualizer plane_normal_visualizer;
		boost::thread visualizer(boost::bind(&PlaneNormalVisualizer::Spin, &(plane_normal_visualizer))); 
		
// 		for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) 
// 		{
// 			laserCloudOri->clear();
// 			coeffSel->clear();
// 			surfPointsCorr[0]->clear(); surfPointsCorr[1]->clear();	
// 			findCorrespondingSurfFeatures(iterCount1);
// 			
// 			// visualize correspondence
// // 			plane_normal_visualizer.UpdateLines(refLidarData.surfPointsLessFlat, dataLidarData.surfPointsLessFlat, surfPointsCorr[0], surfPointsCorr[1]);
// 	
// 			if (laserCloudOri->points.size() < 10)
// 				continue;
// 			if (calculateTransformationSurf(iterCount1) == false)
// 				break;
// 			
// 			std::cout << "iterCound: " << iterCount1 << ": number of coorespondences: " 
// 				<< laserCloudOri->size() << " " << surfPointsCorr[0]->size() << std::endl;
// 			
// 		}
// 		std::cout << "\033[1;32m****************\033[0m AFTER_PLANAR_TRANSFORMATION" << std::endl;
// 		printResult();
	
		for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) 
		{
			laserCloudOri->clear();
			coeffSel->clear();
	
			findCorrespondingCornerFeatures(iterCount2);
	
			// visualize correspondence
			plane_normal_visualizer.UpdateLines(refLidarData.cornerPointsLessSharp, dataLidarData.cornerPointsLessSharp, 
												CornerPointsCorr[0], CornerPointsCorr[1]);			
			
			if (laserCloudOri->points.size() < 10)
				continue;
			if (calculateTransformationCorner(iterCount2) == false)
				break;
			sleep(3);
			std::cout << "iterCound: " << iterCount2 << ": number of coorespondences: " << laserCloudOri->size() << std::endl;
		}
		std::cout << "\033[1;32m****************\033[0m AFTER_CORNER_TRANSFORMATION" << std::endl;
		printResult();

		std::cout << "\033[1;32m****************\033[0m AFTER_TRANSFORMATION" << std::endl;
		printResult();
		
		visualizer.join();
	}

    void TransformToStart(PointType const * const pi, PointType * const po)
    {
		// TODO:
//         float s = 10 * (pi->intensity - int(pi->intensity));
		float s = 1.0;

        float rx = s * transformCur[0];
        float ry = s * transformCur[1];
        float rz = s * transformCur[2];
        float tx = s * transformCur[3];
        float ty = s * transformCur[4];
        float tz = s * transformCur[5];

        float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
        float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
        float z1 = (pi->z - tz);

        float x2 = x1;
        float y2 = cos(rx) * y1 + sin(rx) * z1;
        float z2 = -sin(rx) * y1 + cos(rx) * z1;

        po->x = cos(ry) * x2 - sin(ry) * z2;
        po->y = y2;
        po->z = sin(ry) * x2 + cos(ry) * z2;
        po->intensity = pi->intensity;
		
// 		Eigen::Affine3f temp_tf = Eigen::Affine3f::Identity();
// 		geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum[2], -transformSum[0], -transformSum[1]);
// 		temp_tf.rotate(Eigen::Quaternionf(float(geoQuat.w), float(-geoQuat.y), float(-geoQuat.z), float(geoQuat.x)));
// 		temp_tf.translate(Eigen::Vector3f(transformSum[3], transformSum[4], transformSum[5]));
    }	
	
	void findCorrespondingCornerFeatures(int iterCount)
	{
		int cornerPointsSharpNum = dataLidarData.cornerPointsSharp->points.size();
		for (int i = 0; i < cornerPointsSharpNum; i++) 
		{
			TransformToStart(&dataLidarData.cornerPointsSharp->points[i], &pointSel);
			if (iterCount % 5 == 0) 
			{
				// find closest neighborhood
				kdtreeCornerRef->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis); 
				int closestPointInd = -1, minPointInd2 = -1;    
				
				// find second closest neighborhood
				if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) 
				{
					closestPointInd = pointSearchInd[0];
					int closestPointScan = int(refLidarData.cornerPointsLessSharp->points[closestPointInd].intensity);
					float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist;
					for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) 
					{
						if (int(refLidarData.cornerPointsLessSharp->points[j].intensity) > closestPointScan + 2.5) 
						{
							break;
						}
	
						pointSqDis = (refLidarData.cornerPointsLessSharp->points[j].x - pointSel.x) * 
										(refLidarData.cornerPointsLessSharp->points[j].x - pointSel.x) + 
										(refLidarData.cornerPointsLessSharp->points[j].y - pointSel.y) * 
										(refLidarData.cornerPointsLessSharp->points[j].y - pointSel.y) + 
										(refLidarData.cornerPointsLessSharp->points[j].z - pointSel.z) * 
										(refLidarData.cornerPointsLessSharp->points[j].z - pointSel.z);
	
						if (int(refLidarData.cornerPointsLessSharp->points[j].intensity) > closestPointScan) 
						{
							if (pointSqDis < minPointSqDis2) 
							{
								minPointSqDis2 = pointSqDis;
								minPointInd2 = j;
							}
						}
					}
					for (int j = closestPointInd - 1; j >= 0; j--) 
					{
						if (int(refLidarData.cornerPointsLessSharp->points[j].intensity) < closestPointScan - 2.5) 
						{
							break;
						}
	
						pointSqDis = (refLidarData.cornerPointsLessSharp->points[j].x - pointSel.x) * 
										(refLidarData.cornerPointsLessSharp->points[j].x - pointSel.x) + 
										(refLidarData.cornerPointsLessSharp->points[j].y - pointSel.y) * 
										(refLidarData.cornerPointsLessSharp->points[j].y - pointSel.y) + 
										(refLidarData.cornerPointsLessSharp->points[j].z - pointSel.z) * 
										(refLidarData.cornerPointsLessSharp->points[j].z - pointSel.z);
	
						if (int(refLidarData.cornerPointsLessSharp->points[j].intensity) < closestPointScan) 
						{
							if (pointSqDis < minPointSqDis2) 
							{
								minPointSqDis2 = pointSqDis;
								minPointInd2 = j;
							}
						}
					}
				}
	
				pointSearchCornerInd1[i] = closestPointInd;
				pointSearchCornerInd2[i] = minPointInd2;
			}
	
			if (pointSearchCornerInd2[i] >= 0) 
			{	
				tripod1 = refLidarData.cornerPointsLessSharp->points[pointSearchCornerInd1[i]];
				tripod2 = refLidarData.cornerPointsLessSharp->points[pointSearchCornerInd2[i]];
	
				float x0 = pointSel.x;
				float y0 = pointSel.y;
				float z0 = pointSel.z;
				float x1 = tripod1.x;
				float y1 = tripod1.y;
				float z1 = tripod1.z;
				float x2 = tripod2.x;
				float y2 = tripod2.y;
				float z2 = tripod2.z;
	
				float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
				float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
				float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));
	
				float a012 = sqrt(m11 * m11  + m22 * m22 + m33 * m33);
	
				float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
	
				float la =  ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;
	
				float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;
	
				float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;
	
				float ld2 = a012 / l12;
	
				// TODO: 
				float s = 1;
				if (iterCount >= 5) 
				{
					s = 1 - 1.8 * fabs(ld2);
				}
	
// 				if (s > 0.1 && ld2 != 0) 
				if (ld2 != 0)
				{
					coeff.x = s * la; 
					coeff.y = s * lb;
					coeff.z = s * lc;
					coeff.intensity = s * ld2;
					
					laserCloudOri->push_back(dataLidarData.cornerPointsSharp->points[i]);
					coeffSel->push_back(coeff);
					CornerPointsCorr[0]->push_back(refLidarData.cornerPointsLessSharp->points[pointSearchCornerInd1[i]]);
					CornerPointsCorr[1]->push_back(dataLidarData.cornerPointsSharp->points[i]);					
				}
			}
		}
	}
	
	void findCorrespondingSurfFeatures(int iterCount)
	{
// 		int surfPointsFlatNum = dataLidarData.surfPointsFlat->points.size();
		int surfPointsFlatNum = dataLidarData.surfPointsLessFlat->points.size();		
	
		for (int i = 0; i < surfPointsFlatNum; i++) 
		{
// 			TransformToStart(&dataLidarData.surfPointsFlat->points[i], &pointSel);
			TransformToStart(&dataLidarData.surfPointsLessFlat->points[i], &pointSel);			
			if (iterCount % 5 == 0) 
			{
				// find closest neighborhood
				kdtreeSurfRef->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
				int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
	
				// find second and third closest neighborhood
				if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) 
				{
					closestPointInd = pointSearchInd[0];
					int closestPointScan = int(refLidarData.surfPointsLessFlat->points[closestPointInd].intensity);
	
					float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist, minPointSqDis3 = nearestFeatureSearchSqDist;
					for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) 
					{
						if (int(refLidarData.surfPointsLessFlat->points[j].intensity) > closestPointScan + 2.5) 
						{
							break;
						}
	
						pointSqDis = (refLidarData.surfPointsLessFlat->points[j].x - pointSel.x) * 
										(refLidarData.surfPointsLessFlat->points[j].x - pointSel.x) + 
										(refLidarData.surfPointsLessFlat->points[j].y - pointSel.y) * 
										(refLidarData.surfPointsLessFlat->points[j].y - pointSel.y) + 
										(refLidarData.surfPointsLessFlat->points[j].z - pointSel.z) * 
										(refLidarData.surfPointsLessFlat->points[j].z - pointSel.z);
	
						if (int(refLidarData.surfPointsLessFlat->points[j].intensity) <= closestPointScan) 
						{
							if (pointSqDis < minPointSqDis2) 
							{
								minPointSqDis2 = pointSqDis;
								minPointInd2 = j;
							}
						} else 
						{
							if (pointSqDis < minPointSqDis3) 
							{
								minPointSqDis3 = pointSqDis;
								minPointInd3 = j;
							}
						}
					}
					for (int j = closestPointInd - 1; j >= 0; j--) 
					{
						if (int(refLidarData.surfPointsLessFlat->points[j].intensity) < closestPointScan - 2.5) 
						{
							break;
						}
	
						pointSqDis = (refLidarData.surfPointsLessFlat->points[j].x - pointSel.x) * 
										(refLidarData.surfPointsLessFlat->points[j].x - pointSel.x) + 
										(refLidarData.surfPointsLessFlat->points[j].y - pointSel.y) * 
										(refLidarData.surfPointsLessFlat->points[j].y - pointSel.y) + 
										(refLidarData.surfPointsLessFlat->points[j].z - pointSel.z) * 
										(refLidarData.surfPointsLessFlat->points[j].z - pointSel.z);
	
						if (int(refLidarData.surfPointsLessFlat->points[j].intensity) >= closestPointScan) 
						{
							if (pointSqDis < minPointSqDis2) 
							{
								minPointSqDis2 = pointSqDis;
								minPointInd2 = j;
							}
						} else 
						{
							if (pointSqDis < minPointSqDis3) 
							{
								minPointSqDis3 = pointSqDis;
								minPointInd3 = j;
							}
						}
					}
				}
	
				pointSearchSurfInd1[i] = closestPointInd;
				pointSearchSurfInd2[i] = minPointInd2;
				pointSearchSurfInd3[i] = minPointInd3;
			}
	
			if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) 
			{
	
				tripod1 = refLidarData.surfPointsLessFlat->points[pointSearchSurfInd1[i]];
				tripod2 = refLidarData.surfPointsLessFlat->points[pointSearchSurfInd2[i]];
				tripod3 = refLidarData.surfPointsLessFlat->points[pointSearchSurfInd3[i]];
	
				float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) 
							- (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
				float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) 
							- (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
				float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) 
							- (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
				float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);
	
				float ps = sqrt(pa * pa + pb * pb + pc * pc);
	
				pa /= ps;
				pb /= ps;
				pc /= ps;
				pd /= ps;
	
				float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
	
				// TODO:
				float s = 1;
				if (iterCount >= 5) 
				{
					s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
							+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));
				}
	
				if (s > 0.1 && pd2 != 0) 
				{
					coeff.x = s * pa;
					coeff.y = s * pb;
					coeff.z = s * pc;
					coeff.intensity = s * pd2;
	
// 					laserCloudOri->push_back(dataLidarData.surfPointsFlat->points[i]);
					laserCloudOri->push_back(dataLidarData.surfPointsLessFlat->points[i]);
					coeffSel->push_back(coeff);
					surfPointsCorr[0]->push_back(refLidarData.surfPointsLessFlat->points[pointSearchSurfInd1[i]]);
					surfPointsCorr[1]->push_back(dataLidarData.surfPointsFlat->points[i]);
				}
			}
		}
	}
	
	bool calculateTransformationCorner(int iterCount)
	{
		int pointSelNum = laserCloudOri->points.size();
	
		cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
		cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
		cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
		cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));
	
		float srx = sin(transformCur[0]);
		float crx = cos(transformCur[0]);
		float sry = sin(transformCur[1]);
		float cry = cos(transformCur[1]);
		float srz = sin(transformCur[2]);
		float crz = cos(transformCur[2]);
		float tx = transformCur[3];
		float ty = transformCur[4];
		float tz = transformCur[5];
	
		float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
		float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;
	
		float c5 = crx*srz;
	
		for (int i = 0; i < pointSelNum; i++) 
		{
			pointOri = laserCloudOri->points[i];
			coeff = coeffSel->points[i];
	
			float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
						+ (b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;
	
			float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;
	
			float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;
	
			float d2 = coeff.intensity;
	
			matA.at<float>(i, 0) = ary;
			matA.at<float>(i, 1) = atx;
			matA.at<float>(i, 2) = atz;
			matB.at<float>(i, 0) = -0.05 * d2;
		}
	
		cv::transpose(matA, matAt);
		matAtA = matAt * matA;
		matAtB = matAt * matB;
		cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
	
		if (iterCount == 0) 
		{
			cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
			cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
			cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));
	
			cv::eigen(matAtA, matE, matV);
			matV.copyTo(matV2);
	
			isDegenerate = false;
			float eignThre[3] = {10, 10, 10};
			for (int i = 2; i >= 0; i--) 
			{
				if (matE.at<float>(0, i) < eignThre[i]) 
				{
					for (int j = 0; j < 3; j++) 
					{
						matV2.at<float>(i, j) = 0;
					}
					isDegenerate = true;
				} else 
				{
					break;
				}
			}
			matP = matV.inv() * matV2;
		}
	
		if (isDegenerate) 
		{
			cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
			matX.copyTo(matX2);
			matX = matP * matX2;
		}
	
		transformCur[1] += matX.at<float>(0, 0);
		transformCur[3] += matX.at<float>(1, 0);
		transformCur[5] += matX.at<float>(2, 0);
	
		for(int i=0; i<6; i++)
		{
			if(isnan(transformCur[i]))
				transformCur[i]=0;
		}
	
		float deltaR = sqrt(pow(rad2deg(matX.at<float>(0, 0)), 2));
		float deltaT = sqrt(pow(matX.at<float>(1, 0) * 100, 2) +
							pow(matX.at<float>(2, 0) * 100, 2));
	
		if (deltaR < 0.1 && deltaT < 0.1) 
		{
			return false;
		}
		return true;
	}
	
	bool calculateTransformationSurf(int iterCount)
	{
		int pointSelNum = laserCloudOri->points.size();
	
		cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
		cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
		cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
		cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));
	
		float srx = sin(transformCur[0]);
		float crx = cos(transformCur[0]);
		float sry = sin(transformCur[1]);
		float cry = cos(transformCur[1]);
		float srz = sin(transformCur[2]);
		float crz = cos(transformCur[2]);
		float tx = transformCur[3];
		float ty = transformCur[4];
		float tz = transformCur[5];
	
		float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
		float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
		float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;
	
		float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz;
		float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry;
	
		float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
		float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;
	
		for (int i = 0; i < pointSelNum; i++) 
		{	
			pointOri = laserCloudOri->points[i];
			coeff = coeffSel->points[i];
	
			float arx = (-a1*pointOri.x + a2*pointOri.y + a3*pointOri.z + a4) * coeff.x
						+ (a5*pointOri.x - a6*pointOri.y + crx*pointOri.z + a7) * coeff.y
						+ (a8*pointOri.x - a9*pointOri.y - a10*pointOri.z + a11) * coeff.z;
	
			float arz = (c1*pointOri.x + c2*pointOri.y + c3) * coeff.x
						+ (c4*pointOri.x - c5*pointOri.y + c6) * coeff.y
						+ (c7*pointOri.x + c8*pointOri.y + c9) * coeff.z;
	
			float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;
	
			float d2 = coeff.intensity;
	
			matA.at<float>(i, 0) = arx;
			matA.at<float>(i, 1) = arz;
			matA.at<float>(i, 2) = aty;
			matB.at<float>(i, 0) = -0.05 * d2;
		}
	
		cv::transpose(matA, matAt);
		matAtA = matAt * matA;
		matAtB = matAt * matB;
		cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
	
		if (iterCount == 0) 
		{
			cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
			cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
			cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));
	
			cv::eigen(matAtA, matE, matV);
			matV.copyTo(matV2);
	
			isDegenerate = false;
			float eignThre[3] = {10, 10, 10};
			for (int i = 2; i >= 0; i--) 
			{
				if (matE.at<float>(0, i) < eignThre[i]) 
				{
					for (int j = 0; j < 3; j++) 
					{
						matV2.at<float>(i, j) = 0;
					}
					isDegenerate = true;
				} else 
				{
					break;
				}
			}
			matP = matV.inv() * matV2;
		}
	
		if (isDegenerate) 
		{
			cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
			matX.copyTo(matX2);
			matX = matP * matX2;
		}
	
		transformCur[0] += matX.at<float>(0, 0);
		transformCur[2] += matX.at<float>(1, 0);
		transformCur[4] += matX.at<float>(2, 0);
	
		for(int i=0; i<6; i++)
		{
			if(isnan(transformCur[i]))
				transformCur[i] = 0;
		}
	
		float deltaR = sqrt(pow(rad2deg(matX.at<float>(0, 0)), 2) +
							pow(rad2deg(matX.at<float>(1, 0)), 2));
		float deltaT = sqrt(pow(matX.at<float>(2, 0) * 100, 2));
	
		if (deltaR < 0.1 && deltaT < 0.1) 
		{
			return false;
		}
		return true;
	}
	
	// TODO:
	void printResult()
	{
		// calculate final tf
		transformSum[0] = transformCur[0];
		transformSum[1] = transformCur[1];
		transformSum[2] = transformCur[2];
		transformSum[3] = transformCur[3];
		transformSum[4] = transformCur[4];
		transformSum[5] = transformCur[5];

		geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum[2], -transformSum[0], -transformSum[1]);
		
		estimated_tf.rotate(Eigen::Quaternionf(float(geoQuat.w), float(-geoQuat.y), float(-geoQuat.z), float(geoQuat.x)));
		estimated_tf.translate(Eigen::Vector3f(transformSum[3], transformSum[4], transformSum[5]));
		refined_tf.matrix() = ini_tf.matrix() * estimated_tf.matrix();
		std::cout << "Estimated tf:" << std::endl << estimated_tf.matrix() << std::endl
				<< "Refined tf:" << std::endl << refined_tf.matrix() << std::endl;
	}
	
	void publishCloud()
	{
		// publish final pointcloud
		pcl::PointCloud<PointType> cornerPointsLessSharpTransformed;
		pcl::transformPointCloud(*dataLidarData.cornerPointsLessSharp, cornerPointsLessSharpTransformed, estimated_tf);
		common::publishCloud(pubDataCornerPointsLessSharpTransformed, dataLidarData.header_, cornerPointsLessSharpTransformed);		
		
		pcl::PointCloud<PointType> surfPointsLessFlatTransformed;
		pcl::transformPointCloud(*dataLidarData.surfPointsLessFlat, surfPointsLessFlatTransformed, estimated_tf);
		common::publishCloud(pubDataSurfPointsLessFlatTransformed, dataLidarData.header_, surfPointsLessFlatTransformed);

// 		laserOdometry.header.stamp = cloudHeader.stamp;
// 		laserOdometry.pose.pose.orientation.x = -geoQuat.y;
// 		laserOdometry.pose.pose.orientation.y = -geoQuat.z;
// 		laserOdometry.pose.pose.orientation.z = geoQuat.x;
// 		laserOdometry.pose.pose.orientation.w = geoQuat.w;
// 		laserOdometry.pose.pose.position.x = transformSum[3];
// 		laserOdometry.pose.pose.position.y = transformSum[4];
// 		laserOdometry.pose.pose.position.z = transformSum[5];
// 		pubLaserOdometry.publish(laserOdometry);
	
// 		laserOdometryTrans.stamp_ = dataLidarData.header_.stamp;
// 		laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
// 		laserOdometryTrans.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
// 		tfBroadcaster.sendTransform(laserOdometryTrans);
	}
	
	//////////////////////////////////////////////////////////////
	ros::NodeHandle nh_;
	
	ros::Publisher pubDataCornerPointsLessSharpTransformed;
	ros::Publisher pubDataSurfPointsLessFlatTransformed;
	
	std_msgs::Header cloudHeader;
	
	float transformCur[6];
    float transformSum[6];
	
	pcl::VoxelGrid<PointType> downSizeFilter;
	
	LidarData refLidarData, dataLidarData;
	
	std::vector<pcl::PointCloud<PointType>::Ptr > CornerPointsCorr, surfPointsCorr;
	
	// save the points with coorespondences
	pcl::PointCloud<PointType>::Ptr laserCloudOri; 
	pcl::PointCloud<PointType>::Ptr coeffSel;
	
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerRef;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfRef;
	
	int numCornerRef, numSurfRef;
	
    int pointSelCornerInd[N_SCAN*Horizon_SCAN];
    float pointSearchCornerInd1[N_SCAN*Horizon_SCAN];
    float pointSearchCornerInd2[N_SCAN*Horizon_SCAN];

    int pointSelSurfInd[N_SCAN*Horizon_SCAN];
    float pointSearchSurfInd1[N_SCAN*Horizon_SCAN];
    float pointSearchSurfInd2[N_SCAN*Horizon_SCAN];
    float pointSearchSurfInd3[N_SCAN*Horizon_SCAN];	
	
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

    nav_msgs::Odometry laserOdometry;

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;
	
    bool isDegenerate;
    cv::Mat matP;
		
	Eigen::Affine3f ini_tf, refined_tf, estimated_tf;
};

#endif




