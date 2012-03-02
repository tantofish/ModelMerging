#include "dataReader.h"

void getPointCloud(const cv::Mat &_img, std::vector<cv::Point3f> &_pointCloud){
	float _f = 1.f/575.f;
	for(int j = 0 ; j < 480 ; j++){
		for(int i = 0 ; i < 640 ; i++){
			if(_img.at<unsigned short>(j, i) != 0){
				float z = (float) _img.at<unsigned short>(j, i);
				float x = (320-i)*z*_f;
				float y = (240-j)*z*_f;
				_pointCloud.push_back(cv::Point3f(x,y,z));
			}
		}
	}
	cout << "   point cloud size : " << _pointCloud.size() << endl;
}

void getPointCloud(const cv::Mat &_img, std::vector<cv::Point3f> &_pointCloud, const cv::Mat &_colorImg, std::vector<cv::Vec3f> &_colorCloud, cv::Point3f &center){
	float _f = 1.f/575.82f;
	int count = 0;
	center.x = 0.f;
	center.y = 0.f;
	center.z = 0.f;
	for(int j = 0 ; j < 480 ; j++){
		for(int i = 0 ; i < 640 ; i++){
			if(_img.at<unsigned short>(j, i) != 0){
				float z = (float) _img.at<unsigned short>(j, i);
				float x = (320-i)*z*_f;
				float y = (240-j)*z*_f;
				_pointCloud.push_back(cv::Point3f(x, y, z));

				float b = ((float)_colorImg.at<Vec3b>(j, i)[0])/255.f;
				float g = ((float)_colorImg.at<Vec3b>(j, i)[1])/255.f;
				float r = ((float)_colorImg.at<Vec3b>(j, i)[2])/255.f;
				_colorCloud.push_back(cv::Vec3f(r, g, b));

				count++;
				center.x += x;
				center.y += y;
				center.z += z;
			}
		}
	}
	center.x /= (float)count;
	center.y /= (float)count;
	center.z /= (float)count;
	cout << "   point cloud size : " << _pointCloud.size() << endl;
}

bool readDepthImage(cv::Mat & _img, char* filename){
	ifstream file;
	file.open(filename);
	if(!file.is_open()){
		cout << "open file: " << filename << " failed" << endl;
		return false;
	}
	unsigned short val;
	_img.create(480, 640, CV_16U);
	for(int j = 0 ; j < 480 ; j++)
		for(int i = 0 ; i < 640 ; i++){
			file >> val;
			_img.at<unsigned short>(j, i) = val;
		}
	file.close();

	return true;
}

bool readColorImage(cv::Mat & _img, char* filename){
	_img = imread(filename);
	if(_img.rows != 0)
		return true;
	else{
		cout << "open file: " << filename << " failed" << endl;
		return false;
	}
}