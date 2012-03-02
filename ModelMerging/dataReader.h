#define _CRT_SECURE_NO_WARNINGS
#ifndef DATAREADER_H
#define DATAREADER_H

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
using namespace cv;

void getPointCloud(const cv::Mat &_img, std::vector<cv::Point3f> &_pointCloud);

void getPointCloud(const cv::Mat &_img, std::vector<cv::Point3f> &_pointCloud, 
	const cv::Mat &_colorImg, std::vector<cv::Vec3f> &_colorCloud, cv::Point3f &center);

bool readDepthImage(cv::Mat & _img, char* filename);

bool readColorImage(cv::Mat & _img, char* filename);



#endif