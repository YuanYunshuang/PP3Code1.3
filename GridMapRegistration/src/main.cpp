#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>
#include <limits>
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> XYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZ_p;

vector<string> getFiles(string cate_dir)
{
	int len;
	vector<string> files;//container for file names 
	struct dirent *pDirent;
	DIR *pDir;

	pDir = opendir(cate_dir.c_str());
	if (pDir == NULL) {
		printf("Cannot open directory '%s'\n", cate_dir.c_str());
	}

	while ((pDirent = readdir(pDir)) != NULL) {
		//cout<< string(pDirent->d_name)<<endl;
		if ((pDirent->d_name[0]) != '.')
			files.push_back(string(pDirent->d_name));
	}
	closedir(pDir);
	//sort(files.begin(), files.end());
	return files;
}



XYZ_p filterPointCloud(XYZ_p cloud){
	XYZ_p cloud_filtered(new XYZ);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.1f, 0.1f, 0.1f);
	sor.filter(*cloud_filtered);
	
	return cloud_filtered;
}

cv::Mat densityMap(XYZ_p cloud, double resolution, cv::Mat img) {

	double min_z = numeric_limits<double>::max();
	pcl::PointXYZ center(0,0,0);
	for (int i = 0; i < cloud->points.size(); i++) {
		center.x += cloud->points[i].x;
		center.y += cloud->points[i].y;
		center.z += cloud->points[i].z;
//		if (cloud->points[i].z < min_z) {
//			min_z = cloud->points[i].z;
//		}
	}
	center.x /= cloud->points.size();
	center.y /= cloud->points.size();
	center.z/=cloud->points.size();
	cout << min_z << endl;
	cout << cloud->points.size() << endl;
	for (int i = 0; i < cloud->points.size(); i++) {
		pcl::PointXYZ p = cloud->points[i];
		cv::Point2d ip((p.x - center.x) / resolution + img.cols / 2,
				img.rows / 2 - (p.y - center.y) / resolution);
		if (p.z > center.z&&p.z<center.z+1 && ip.x < img.cols && ip.y < img.rows && ip.x > 0
				&& ip.y > 0) {
			//cout<<int(ip.x)<<","<<int(ip.y)<<endl;
			img.at<char>(ip.x, ip.y) += 50;
			if(img.at<unsigned char>(ip.x, ip.y)>255)
				img.at<unsigned char>(ip.x, ip.y) = 255;
		}
	}
	/*for(int i=0; i<img.rows; i+=10){
		for(int j=0; j<img.cols; j+=10)
			cout<<int(img.at<char>(i, j))<<",";
		cout<<endl;
	}*/
	//cout<<img.rows<<","<<img.cols<<endl;
	//cv::namedWindow( "Display window",cv:: WINDOW_AUTOSIZE );
	cv::imshow("image",img);

	return img;
}

int main(int argc, char ** argv) {
	if (argc <3) {
		std::cout << "Need one path to pointcloud, and one path for saving images" << std::endl;
	}
	string pt_path = argv[1];
	string im_path = argv[2];
	//get the file paths of the point clouds
	vector<string> files = getFiles(pt_path);

	//implemeting for each file: translate point cloud into 2D image
	XYZ_p cloud(new XYZ);
	pcl::PLYReader Reader;
	string path;
	for(int i=0; i<files.size(); i++){
		cout<<pt_path +"/"+ files[i]<<endl;
		Reader.read(pt_path +"/"+ files[i], *cloud);
		cout << cloud->points.size() << endl;
		//XYZ_p cloud_filtered = filterPointCloud(cloud);
		double resolution = 0.1; //one pixel =0.1m
		cv::Mat img(1000, 1000, CV_8UC1, cv::Scalar(0));
		img = densityMap(cloud, resolution, img);
		cv::waitKey(0);
		path = files[i];
		path = path.substr(0, path.length() - 4);
		path = im_path + "/images/" + path + ".jpg";
		cout<<"Saving image:"<<path<<endl;
		cv::imwrite(path,img);
	}

	return 1;
}
