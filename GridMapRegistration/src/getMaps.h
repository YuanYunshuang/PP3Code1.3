
#ifndef __GETMAPS_H__
#define __GETMAPS_H__

#include "./utils.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/flann.hpp>
#include <math.h> 


namespace pp3{

static double resolution = 0.1;

void densityMap(PointCloud &pt, int im_size) {

	//double min_z = numeric_limits<double>::max();
	cv::Mat img(im_size, im_size, CV_8UC1, cv::Scalar(0));
	
	for (int i = 0; i < pt.cloud->points.size(); i++) {
		pcl::PointXYZ p = pt.cloud->points[i];
		cv::Point2d ip((p.x - pt.center.x) / resolution + img.cols / 2,
				img.rows / 2 - (p.y - pt.center.y) / resolution);
		if (p.z > pt.floor_height && ip.x < img.cols && ip.y < img.rows && ip.x > 0
				&& ip.y > 0) {
			img.at<char>(ip.x, ip.y) += 20;
			if(img.at<char>(ip.x, ip.y)>30)
				img.at<char>(ip.x, ip.y) = 255;
		}
	}
	pt.density_map = img;
	//cv::namedWindow( "Display window",cv:: WINDOW_AUTOSIZE );
	/*cv::imshow("density map",pt.density_map);
	cv::waitKey(0);
	cv::destroyAllWindows();*/
}

void depthMap(PointCloud &pt, int im_size) {

	cv::Mat img(im_size, im_size, CV_64F, cv::Scalar(0.0));
	cv::Mat count(im_size, im_size, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < pt.cloud->points.size(); i++) {
		pcl::PointXYZ p = pt.cloud->points[i];
		//cout<<p.x<<","<<p.y<<","<<p.z<<endl;
		p.z = p.z - pt.floor_height;
		cv::Point2d ip((p.x - pt.center.x) / resolution + img.cols / 2,
				img.rows / 2 - (p.y - pt.center.y) / resolution);
		if (p.z>0 && p.z<3 && ip.x < img.cols && ip.y < img.rows && ip.x > 0
				&& ip.y > 0) {
			img.at<double>(ip.x, ip.y) = fmax(img.at<double>(ip.x, ip.y),p.z);
			//img.at<double>(ip.x, ip.y) += p.z;
			//count.at<char>(ip.x, ip.y) += 1 ;
		}
	}

	/*for( int j = 0; j < count.rows ; j++ ){
		 for( int i = 0; i < count.cols; i++ ){
			if(count.at<char>(j, i)>0){
				img.at<double>(j, i) /= (double)count.at<char>(j, i);
			}
			//cout<<(int)img.at<double>(j, i)<<",";
		 }
		//cout<<endl;
	}*/
	cv::normalize(img, pt.depth_map, 0, 255, cv::NORM_MINMAX,CV_8UC1, cv::Mat() );
	
	//cv::namedWindow( "depth map",cv:: WINDOW_AUTOSIZE );
	/*cv::imshow("depth map",pt.depth_map);
	cv::waitKey(0);
	cv::destroyAllWindows();*/
}

void planarMap(PointCloud &pt, int num_nbs, float curvature_thr, int im_size){
	XYZ_p cloud(new XYZ);
	cloud = pt.cloud;
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 50cm
	ne.setKSearch (num_nbs);
	// Compute the features
	ne.compute (*cloud_normals);
	//debug: print normals
	cv::Mat img(im_size, im_size,CV_8UC1, cv::Scalar(0.0));
	//cv::Mat count(700, 700, CV_8UC1, cv::Scalar(0));

	for(int i=0; i<cloud_normals->points.size(); i++){
		//cout<<cloud_normals->points[i]<<endl;		
		pcl::PointXYZ p = pt.cloud->points[i];
		p.z = p.z - pt.floor_height;
		cv::Point2d ip((p.x - pt.center.x) / resolution + img.cols / 2,
				img.rows / 2 - (p.y - pt.center.y) / resolution);
		if (p.z>0 && p.z<2 && ip.x < img.cols && ip.y < img.rows && ip.x > 0 && ip.y > 0) {
			cout<<cloud_normals->points[i]<<endl;
			float curv = fabs(cloud_normals->points[i].curvature);
			//cout<<curv<<endl;
			if(curv<curvature_thr)
				img.at<char>(ip.x, ip.y) += 1;
				//count.at<char>(ip.x, ip.y) += 1 ;
		}
	
	}
	cv::normalize(img, pt.planar_map, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
	cv::imshow("cuvature map",pt.planar_map);
	cv::waitKey(0);
	cv::destroyAllWindows();
}

void deffNormalsMap(PointCloud &pt, double scale1, double scale2, double don_thr,int im_size){
	XYZ_p cloud(new XYZ);
	cloud = pt.cloud;

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
	ne.setInputCloud (cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
	// calculate normals with the small scale
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);
	ne.setRadiusSearch (scale1);
	ne.compute (*normals_small_scale);
	// calculate normals with the large scale
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);
	ne.setRadiusSearch (scale2);
	ne.compute (*normals_large_scale);

	// Create output cloud for DoN results
	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud, *doncloud);

	cout << "Calculating DoN... " << endl;
	// Create DoN operator
	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
	don.setInputCloud (cloud);
	don.setNormalScaleLarge (normals_large_scale);
	don.setNormalScaleSmall (normals_small_scale);
	if (!don.initCompute ())
	  {
	    std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
	    exit (EXIT_FAILURE);
	  }

	// Compute DoN
	don.computeFeature (*doncloud);

	cv::Mat img(im_size, im_size,CV_8UC1, cv::Scalar(0));

	for(int i=0; i<doncloud->points.size(); i++){
		//cout<<doncloud->points[i]<<endl;
		auto p = doncloud->points[i];
		p.z = p.z - pt.floor_height;
		cv::Point2d ip((p.x - pt.center.x) / resolution + img.cols / 2,
				img.rows / 2 - (p.y - pt.center.y) / resolution);
		if (p.z>0 && p.z<3 && ip.x < img.cols && ip.y < img.rows && ip.x > 0 && ip.y > 0) {
			//cout<<doncloud->points[i].data_c[0]<<","<<doncloud->points[i].data_c[1]<<",";
			//cout<<doncloud->points[i].data_c[2]<<","<<doncloud->points[i].data_c[1]<<endl;
			float don = fabs(doncloud->points[i].curvature);
			//cout<<don<<endl;
			if(don<don_thr && don>0.0){
				if(img.at<uchar>(ip.x, ip.y)<100)
					img.at<uchar>(ip.x, ip.y) += 100;
				else
					img.at<uchar>(ip.x, ip.y) = 255;
				//count.at<char>(ip.x, ip.y) += 1 ;
			}
		}

	}

	//cv::normalize(img, pt.diff_normals_map, 0, 255, cv::NORM_MINMAX,CV_8UC1, cv::Mat());

	pt.diff_normals_map = img;

	cv::imshow("Difference of Normals",pt.diff_normals_map);
	cv::waitKey(0);
	cv::destroyAllWindows();
}

void PCA2d(cv::Mat &img_in, cv::Mat &img_out, int num, double thr){
	//cv::threshold(img_in, img_out, 1, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	img_out = img_in;
	pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);
	vector<pair<int,int>> points;
	for(int i=0;i<img_in.rows;i++){
		for(int j=0;j<img_in.cols;j++){
			if(img_in.at<uchar>(i,j)>0){
				points.emplace_back(pair<int,int>(i,j));
			}
		}
	}

	// Generate 2d pointcloud data
	cloud->width = points.size();
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);

	for (size_t i = 0; i < points.size (); i++)
	{
		cloud->points[i].x = points[i].first;
		cloud->points[i].y = points[i].second;
	}

	pcl::KdTreeFLANN<pcl::PointXY> kdtree;

	kdtree.setInputCloud (cloud);

	for(size_t i = 0; i < cloud->points.size (); i++){
		pcl::PointXY searchPoint;
		searchPoint = cloud->points[i];
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;


//		std::cout << "Neighbors within radius search at (" << searchPoint.x
//				<< " " << searchPoint.y
//				<< ") with radius=" << radius << std::endl;


		kdtree.radiusSearch (searchPoint, num, pointIdxRadiusSearch, pointRadiusSquaredDistance);
//		{
//		for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
//		  std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[j] ].x
//					<< " " << cloud->points[ pointIdxRadiusSearch[j] ].y
//					<< " (squared distance: " << pointRadiusSquaredDistance[j] << ")" << std::endl;
//		}


		int sz = pointIdxRadiusSearch.size ();
		cv::Mat data_pts = cv::Mat(sz, 2, CV_32F);
		for (int j = 0; j < sz; j++)
		{
			data_pts.at<float>(j,0)=cloud->points[ pointIdxRadiusSearch[j] ].x;
			data_pts.at<float>(j,1)=cloud->points[ pointIdxRadiusSearch[j] ].y;
		}
		//int stop;
		//cin>>stop;

		//Perform PCA analysis
		cv::PCA pca_analysis(data_pts,cv:: Mat(), cv::PCA::DATA_AS_ROW);
		//Store the center of the object
		cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
						  static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
		//Store the eigenvalues and eigenvectors
		// std::vector<cv::Point2d> eigen_vecs(2);
		std::vector<float> eigen_val(2);
		for (int k = 0; k < 2; k++)
		{
		  //  eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
		  //                         pca_analysis.eigenvectors.at<double>(i, 1));
			eigen_val[k] = pca_analysis.eigenvalues.at<float>(k);
		}
		//cout<<eigen_val[0]<<","<<eigen_val[1]<<endl;
		float compactness = fabs(fabs(eigen_val[0])/fabs(eigen_val[1]+eigen_val[0])-0.5);
		cout<<compactness<<endl;
		if(compactness>thr){
			img_out.at<uchar>(points[i].first,points[i].second) = 255;
		}
		else{
			img_out.at<uchar>(points[i].first,points[i].second) = 0;
		}


	}
}

void NonMaxSurpression(cv::Mat &img_in, cv::Mat &img_out, double r, int thr){
	//cv::threshold(img_in, img_out, 1, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	img_out = img_in;
	pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);
	vector<pair<int,int>> points;
	for(int i=0;i<img_in.rows;i++){
		for(int j=0;j<img_in.cols;j++){
			if(img_in.at<uchar>(i,j)>0){
				points.emplace_back(pair<int,int>(i,j));
			}
		}
	}

	// Generate 2d pointcloud data
	cloud->width = points.size();
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);

	for (size_t i = 0; i < points.size (); i++)
	{
		cloud->points[i].x = points[i].first;
		cloud->points[i].y = points[i].second;
	}

	pcl::KdTreeFLANN<pcl::PointXY> kdtree;

	kdtree.setInputCloud (cloud);

	for(size_t i = 0; i < cloud->points.size (); i++){
		pcl::PointXY searchPoint;
		searchPoint = cloud->points[i];
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		kdtree.radiusSearch (searchPoint, r, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		int count = 0;
		int sz = pointIdxRadiusSearch.size ();
		for (int j = 0; j < sz; j++)
		{
			if(img_in.at<uchar>(cloud->points[ pointIdxRadiusSearch[j] ].x,
					cloud->points[ pointIdxRadiusSearch[j] ].y)>250){
				count++;
			}
		}
		//cout<<compactness<<endl;
		if(count<thr){
			img_out.at<uchar>(points[i].first,points[i].second) = 0;
		}
		else{
			img_out.at<uchar>(points[i].first,points[i].second) = 255;
		}
	}

}

}//end namespace pp3

#endif
