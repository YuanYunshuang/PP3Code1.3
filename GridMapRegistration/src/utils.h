#ifndef __UTILS_H__
#define __UTILS_H__


//#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>
#include <limits>


namespace pp3{
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



void filterPointCloud(XYZ_p &cloud){
	XYZ_p cloud_filtered(new XYZ);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.2f, 0.2f, 0.2f);
	sor.filter(*cloud_filtered);

	cloud = cloud_filtered;
	cout << "After filtering: "<<cloud->points.size() << endl;
}

void calCenter(const XYZ_p &cloud, pcl::PointXYZ &center){
		
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
}

double calFloorHeight(const XYZ_p &cloud, pcl::PointXYZ center, double thickness){
	double height = center.z;
	std::vector<std::pair<double, int>> count;	
	for(double h=-2.0; h<2.0; h+=0.1){
		int ct = 0;
		for (int i = 0; i < cloud->points.size(); i++) {
			if(cloud->points[i].z > (height-h-thickness) && cloud->points[i].z < (height-h+thickness))
				ct++;
		}
		std::pair<double, int> p(height-h,ct);
		count.push_back(p);
	}

	std::vector<std::pair<double, int>>::iterator max_it = std::max_element(count.begin(),count.end(),
					[] (std::pair<double, int> const& c1, std::pair<double, int> const& c2)
					{return c1.second<c2.second;});
	std::pair<double, int> result = count[std::distance(std::begin(count),max_it)]; 
	std::cout<<"Floor height, num_inliers: "<<result.first<<","<<result.second<<std::endl;

	return height;
}

}//end namespace pp1.3




#endif
