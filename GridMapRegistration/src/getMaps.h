
#ifndef __GETMAPS_H__
#define __GETMAPS_H__

#include "./utils.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
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
	cv::normalize(img, pt.depth_map, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
	
	//cv::namedWindow( "depth map",cv:: WINDOW_AUTOSIZE );
	/*cv::imshow("depth map",pt.depth_map);
	cv::waitKey(0);
	cv::destroyAllWindows();*/
}

void planarMap(PointCloud &pt, double radius, float curvature_thr, int im_size){
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
	ne.setRadiusSearch (radius);
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
		if (p.z>0 && p.z<5 && ip.x < img.cols && ip.y < img.rows && ip.x > 0 && ip.y > 0) {
			float curv = fabs(cloud_normals->points[i].curvature);
			//cout<<curv<<endl;
			if(curv<curvature_thr)
				img.at<char>(ip.x, ip.y) += 1;
				//count.at<char>(ip.x, ip.y) += 1 ;
		}
	
	}
	cv::normalize(img, pt.planar_map, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
	/*cv::imshow("cuvature map",pt.planar_map);
	cv::waitKey(0);
	cv::destroyAllWindows();*/
}

}//end namespace pp3

#endif
