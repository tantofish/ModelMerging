#ifndef MODELMAKER_H
#define MODELMAKER_H

#include <windows.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <iterator>
using namespace std;

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
using namespace cv;

#include <GL\glut.h>
#include "dataReader.h"
#include "myTimer.h"

#define PI 3.141592653589793
#define toRad(x) ((x)*0.01745329251994)
#define toDeg(x) ((x)*57.2957795130823)


class ModelMaker{
public:
	ModelMaker(): mtPitch(0.f), mtYaw(0.f), mtRoll(0.f), 
		mtTransX(0.f), mtTransY(0.f), mtTransZ(0.f), mtRStep(1.f), mtTStep(2.f), 
		colorMode(0.f), showSrc(true), showDst(true), showReadModel(false), doMatching(false), 
		isKey(false), bestMatchStableCounter(0){

		gradient.push_back(Vec6f(0.f, 0.f, 0.f, 0.f, 0.f, 0.f));
		gradient.push_back(Vec6f(1.f, 0.f, 0.f, 0.f, 0.f, 0.f));
		gradient.push_back(Vec6f(-1.f, 0.f, 0.f, 0.f, 0.f, 0.f));
		gradient.push_back(Vec6f(0.f, 1.f, 0.f, 0.f, 0.f, 0.f));
		gradient.push_back(Vec6f(0.f, -1.f, 0.f, 0.f, 0.f, 0.f));
		gradient.push_back(Vec6f(0.f, 0.f, 1.f, 0.f, 0.f, 0.f));
		gradient.push_back(Vec6f(0.f, 0.f, -1.f, 0.f, 0.f, 0.f));
		gradient.push_back(Vec6f(0.f, 0.f, 0.f, 1.f, 0.f, 0.f));
		gradient.push_back(Vec6f(0.f, 0.f, 0.f, -1.f, 0.f, 0.f));
		gradient.push_back(Vec6f(0.f, 0.f, 0.f, 0.f, 1.f, 0.f));
		gradient.push_back(Vec6f(0.f, 0.f, 0.f, 0.f, -1.f, 0.f));
		gradient.push_back(Vec6f(0.f, 0.f, 0.f, 0.f, 0.f, 1.f));
		gradient.push_back(Vec6f(0.f, 0.f, 0.f, 0.f, 0.f, -1.f));

		
		//amList.push_back(pair<int,int>(27,44));
		//amList.push_back(pair<int,int>(2,16));
		//amList.push_back(pair<int,int>(57,72));
		amList.push_back(pair<int,int>(87,102));
		amPIndex = 0;	
		amFIndex = amList[amPIndex].first;
		amDone = false;
		amInit = false;
	}
	void readData(int fileIndex, bool isSource);
	void buildTree(std::vector<cv::Point3f> &_pCloud);
	void queryTree();
	void drawPointCloud(const std::vector<cv::Point3f> &pCloud, const std::vector<cv::Vec3f> &cCloud);

	/* Apply Translation and Rotation to _pCloud and save the result to _pCloudRes */
	void transformPointCloud(
		cv::Vec6f arguments,
		const std::vector<cv::Point3f> &_pCloud, const cv::Point3f &_center,
		std::vector<cv::Point3f> &_pCloudRes, cv::Point3f &_centerRes);

	/* peak the 13 gradient energys */
	int computeAllGradientEnergys();

	/* count inlier */
	int errFuncInlier(const std::vector<cv::Point3f> &_pCloud, const int step);

	/* sum of all distances */
	float errFuncDist(const std::vector<cv::Point3f> &_pCloud, const int step);
	
	/* automatically match src to dst, the result is a 6D vector (parameters)*/
	bool matchProcess();


	/* when the solution arguments are confirmed, you need to merge transCloud into dstPointCloud*/
	void mergeSrcDstPClouds();

	/* keyboard function*/
	void registerKey(int key);

	


	bool isKey;
	int fileIndex;
	MYTimer timer;

	// point cloud used for source cloud geometrical transformation
	std::vector<cv::Point3f> transCloud;
	cv::Point3f transCenter;

	// point cloud used for gradient energy peaking transCloud transformation
	std::vector<cv::Point3f> bufCloud;
	cv::Point3f bufCenter;

	// source point cloud
	std::vector<cv::Point3f> srcPointCloud;
	std::vector<cv::Vec3f>	 srcColorCloud;
	cv::Point3f srcCenter;

	// destination point cloud
	std::vector<cv::Point3f> dstPointCloud;
	std::vector<cv::Vec3f>	 dstColorCloud;
	cv::Point3f dstCenter;

	//
	std::vector<cv::Point3f> inPointCloud;
	std::vector<cv::Vec3f>	 inColorCloud;
	cv::Point3f inCenter;


	// model control variables
	float mtPitch;
	float mtYaw;
	float mtRoll;
	float mtTransX;
	float mtTransY;
	float mtTransZ;

	float mtRStep;
	float mtTStep;

	int colorMode;	// 0=true color; 1=red and green, 2=depth visualize(not yet implemented)
	
	bool showSrc, showDst, showReadModel;

	std::vector<cv::Vec6f> gradient;

	/* automatic do merge */
	void automaticMerge();


	private:

	/* output .obj*/
	void mWriteObj(char* filename);
	/* output .tri*/
	void mWriteTri(char* filename);
	/* output "%s.txt" */
	bool mWriteModel();
	/* read output "%s.txt" */
	bool mReadModel();
	/* alpha blending on/off switch */
	void alphaBlendingSwitch();
	/* select frame to load */
	void selectFrame();

	cv::flann::Index kdtree;
	
	cv::Mat srcD, dstD;
	cv::Mat srcI, dstI;

	bool doMatching;
	int bestMatchStableCounter;


	
	Vector<pair<int,int>> amList;
	int amPIndex;	// pair index
	int amFIndex;	// frame index
	bool amDone;
	bool amInit;
};

#endif