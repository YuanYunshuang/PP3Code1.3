#include "./utils.h"

//using namespace std;
using namespace pp3;




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
			img.at<char>(ip.x, ip.y) += 50;
			if(img.at<char>(ip.x, ip.y)>255)
				img.at<char>(ip.x, ip.y) = 255;
		}
	}

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
