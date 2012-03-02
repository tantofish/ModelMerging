#include "modelMaker.h"

//test
#include <omp.h>

void ModelMaker::readData(int fileIndex, bool isSource){
	char dFilePath[30], iFilePath[30];
	sprintf(dFilePath, "data\\depthraw\\%d_raw.txt", fileIndex);
	sprintf(iFilePath, "data\\color\\%d_color.png", fileIndex);

	if(isSource){
		cout << "Reading Source Image "<< fileIndex << "..." ;
		readDepthImage(srcD, dFilePath);
		readColorImage(srcI, iFilePath);
		cout << " Source Point Cloud..." ;
		srcPointCloud.clear();
		srcColorCloud.clear();
		getPointCloud(srcD, srcPointCloud, srcI, srcColorCloud, srcCenter);
	}else{
		cout << "Reading Destination Image "<< fileIndex << "..." ;
		dstPointCloud.clear();
		dstColorCloud.clear();
		readDepthImage(dstD, dFilePath);
		readColorImage(dstI, iFilePath);
		cout << " Point Cloud...";
		getPointCloud(dstD, dstPointCloud, dstI, dstColorCloud, dstCenter);
	}
}

void ModelMaker::buildTree(std::vector<cv::Point3f> &_pCloud){
	cout << "Building Destination Tree..." << endl;
	cv::flann::KDTreeIndexParams indexParams;
	kdtree.build(cv::Mat(_pCloud).reshape(1), indexParams);
}

/* return value is the best match index */
int ModelMaker::computeAllGradientEnergys(){
	Vec6f paraCurrent(mtYaw, mtPitch, mtRoll, mtTransX, mtTransY, mtTransZ);
	Vec6f paraPeek;

	int max = INT_MIN;
	float min = FLT_MAX;
	int idx = -1;
	int inlier;
	float dist;
	for(int i = 0 ; i < 13 ; i ++){
		if(i > 6)
			for(int j = 0 ; j < 6 ; j++)	paraPeek[j] = paraCurrent[j] + gradient[i][j] * mtTStep;
		else if(i > 0)
			for(int j = 0 ; j < 6 ; j++)	paraPeek[j] = paraCurrent[j] + gradient[i][j] * mtRStep;
		else
			for(int j = 0 ; j < 6 ; j++)	paraPeek[j] = paraCurrent[j];
		transformPointCloud(paraPeek, this->srcPointCloud, this->srcCenter, this->bufCloud, this->bufCenter); 
		//printf("(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", paraPeek[0], paraPeek[1], paraPeek[2], paraPeek[3], paraPeek[4], paraPeek[5]);
		
		
		dist = errFuncDist(this->bufCloud, 20);
		if( dist < min ){
			min = dist;
			idx = i;
		}

		/*	use inliner count for energy function
		inlier = errFuncInlier(this->bufCloud, 20);
		if( inlier > max ){
			max = inlier;
			idx = i;
		}
		*/
	}
	//printf("= = = = MAX = = = =\n");

	float step;
	step=(idx>6)?mtTStep:mtRStep;
	mtYaw	 += gradient[idx][0] * step;
	mtPitch	 += gradient[idx][1] * step;
	mtRoll	 += gradient[idx][2] * step;
	mtTransX += gradient[idx][3] * step;
	mtTransY += gradient[idx][4] * step;
	mtTransZ += gradient[idx][5] * step;

	//printf("(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f) : %d\n", mtYaw, mtPitch, mtRoll, mtTransX, mtTransY, mtTransZ, max);	
	printf("(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f) : %.2f  ", mtYaw, mtPitch, mtRoll, mtTransX, mtTransY, mtTransZ, min);	
	return idx;
}
/*
 * Count number of inliers which are within a given distance threshold
 * but still easy to be dominated by miss match points
 */
int ModelMaker::errFuncInlier(const std::vector<cv::Point3f> &_pCloud, const int step){
	int nSamples = 0;
	int inlierCount = 0;
	float distSum = 0.f;
	float distThresh = 10.f;
	for(unsigned int queryIdx = 0 ; queryIdx < _pCloud.size() ; queryIdx += step){
		float x = _pCloud[queryIdx].x;
		float y = _pCloud[queryIdx].y;
		float z = _pCloud[queryIdx].z;
		//printf("Now Querying (%.2f, %.2f, %.2f)\n", x, y, z);
		vector<float> query;
		query.push_back(x);
		query.push_back(y);
		query.push_back(z);
	
		int knn = 1;
		vector<int> indices(knn);
		vector<float> dists(knn);
		
		
		//timer.timeInit();
		kdtree.knnSearch(query, indices, dists, knn, flann::SearchParams(32));
		//timer.timeReport();
		
		/*int idx;double dist;
		idx = indices[0];
		dist = dists[0];
		printf("query result (%.2f, %.2f, %.2f)", 
			pointCloudA[idx].x, pointCloudA[idx].y, pointCloudA[idx].z);
		printf("	index : %d,	dist : %.2f\n", idx, dist);*/
		
		nSamples++;
		if(dists[0] <= distThresh){
			inlierCount++;
			distSum += dists[0];
		}
	}

	//printf("par[6]=(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)	inlier: %d\n", mtYaw, mtPitch, mtRoll, mtTransX, mtTransY, mtTransZ, inlierCount);
	//printf("inlier: %d, distSum: %.2f, nSamples: %d\n", inlierCount, distSum, nSamples);
	return inlierCount;
}

float ModelMaker::errFuncDist(const std::vector<cv::Point3f> &_pCloud, const int step){
	int nSamples = 0;
	int inlierCount = 0;
	float distSum = 0.f;
	//float inlierDistSum = 0.f;
	float distThresh = 10.f;
	
//#pragma omp parallel for reduction(+:distSum)
	for(int queryIdx = 0 ; queryIdx < _pCloud.size() ; queryIdx += step){
		
		vector<float> query(3);
		query[0] = _pCloud[queryIdx].x;
		query[1] = _pCloud[queryIdx].y;
		query[2] = _pCloud[queryIdx].z;

		int knn = 1;
		vector<int> indices(knn);
		vector<float> dists(knn);

		//timer.timeInit();
		kdtree.knnSearch(query, indices, dists, knn, flann::SearchParams(32));
		//timer.timeReport();
		
		
		/*int idx;double dist;
		idx = indices[0];
		dist = dists[0];
		printf("query result (%.2f, %.2f, %.2f)", 
			pointCloudA[idx].x, pointCloudA[idx].y, pointCloudA[idx].z);
		printf("	index : %d,	dist : %.2f\n", idx, dist);*/
		
		//nSamples++;
		distSum = distSum + dists[0];
		//if(dists[0] <= distThresh){
			//inlierCount++;
			//inlierDistSum += dists[0];
		//}
	}

	//if( (float)inlierCount/(float)nSamples >= 0.6)
		//return inlierDistSum/(float)inlierCount;

	//printf("par[6]=(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)	inlier: %d\n", mtYaw, mtPitch, mtRoll, mtTransX, mtTransY, mtTransZ, inlierCount);
	//printf("inlier: %d, distSum: %.2f, nSamples: %d\n", inlierCount, distSum, nSamples);
	return distSum;
}

bool ModelMaker::matchProcess(){
	if(!doMatching) return false;
	

	int bestMatchIdx = computeAllGradientEnergys();
	printf("index = %d\n", bestMatchIdx);
	if(bestMatchIdx == 0){
		if(bestMatchStableCounter>=2 || mtRStep<=0.04){
			printf(" matching converged! \n");
			bestMatchStableCounter = 0;
			mtRStep = 1.f;
			mtTStep = 2.f;
			doMatching = false;
		}
		bestMatchStableCounter++;
		mtRStep*=0.5;
		mtTStep*=0.5;
	}else{
		bestMatchStableCounter = 0;
	}

	return true;
}

void ModelMaker::drawPointCloud(const std::vector<cv::Point3f> &pCloud, const std::vector<cv::Vec3f> &cCloud){
	//glEnable(GL_POINT_SMOOTH);
	//glPointSize(2.0);
	glBegin(GL_POINTS);
	switch(colorMode){
	case 0:
		for(unsigned int i = 0 ; i < pCloud.size() ; i++){
			glColor4f(cCloud[i][0], cCloud[i][1], cCloud[i][2], 0.5f);
			glVertex3f(pCloud[i].x, pCloud[i].y, pCloud[i].z);
		}
		break;
	case 1:
		if(pCloud.size() == this->dstPointCloud.size())
			for(unsigned int i = 0 ; i < pCloud.size() ; i++){
				glColor4f(1.f, cCloud[i][1], cCloud[i][2], 0.5f);
				glVertex3f(pCloud[i].x, pCloud[i].y, pCloud[i].z);
			}
		else
			for(unsigned int i = 0 ; i < pCloud.size() ; i++){
				glColor4f(cCloud[i][0], 1.f, cCloud[i][2], 0.5f);
				glVertex3f(pCloud[i].x, pCloud[i].y, pCloud[i].z);
			}
		break;
	}
	
	glEnd();
}

void ModelMaker::transformPointCloud(const cv::Vec6f arguments,
									 const std::vector<cv::Point3f> &_pCloud, const cv::Point3f &_center,
									 std::vector<cv::Point3f> &_pCloudRes, cv::Point3f &_centerRes){
	
	
	if(_pCloudRes.size() != _pCloud.size())	_pCloudRes.resize(_pCloud.size());

	int nPoints = (int) _pCloud.size();
	float yaw   = toRad(arguments[0]);
	float pitch = toRad(arguments[1]);
	float roll  = toRad(arguments[2]);
	float tx	= arguments[3];
	float ty	= arguments[4];
	float tz	= arguments[5];

	float tmpX, tmpY, tmpZ;
	float X, Y, Z;
	
	for(int i = 0 ; i < nPoints ; i++){
		// align center to (0, 0 ,0)
		X = _pCloud[i].x - _center.x;
		Y = _pCloud[i].y - _center.y;
		Z = _pCloud[i].z - _center.z;

		// Simulate glRotatef(roll, 0, 0, 1)
		tmpX = X * cos(roll) - Y * sin(roll);
		tmpY = X * sin(roll) + Y * cos(roll);
		tmpZ = Z;
		X = tmpX;	Y = tmpY;	Z = tmpZ;

		// Simulate glRotatef(yaw, 0, 1, 0)
		tmpX =  X * cos(yaw) + Z * sin(yaw);
		tmpY =  Y ;
		tmpZ = -X * sin(yaw) + Z * cos(yaw);
		X = tmpX;	Y = tmpY;	Z = tmpZ;

		// Simulate glRotatef(pitch, 1, 0, 0)
		tmpX = X ;
		tmpY = Y * cos(pitch) - Z * sin(pitch);
		tmpZ = Y * sin(pitch) + Z * cos(pitch);

		// Simulate glTranslatef(tx, ty, yz)  +  align to original center
		_pCloudRes[i].x = tmpX + _center.x + tx;
		_pCloudRes[i].y = tmpY + _center.y + ty;
		_pCloudRes[i].z = tmpZ + _center.z + tz;
		_centerRes.x = _center.x + tx;
		_centerRes.y = _center.y + ty;
		_centerRes.z = _center.z + tz;
	}
}

void ModelMaker::mergeSrcDstPClouds(){

	// about 10ms
	/*for(unsigned int i = 0 ; i < srcPointCloud.size() ; i++){
		dstPointCloud.push_back(transCloud[i]);
		dstColorCloud.push_back(srcColorCloud[i]);
	}*/
	// about 1ms
	dstPointCloud.insert(dstPointCloud.end(), transCloud.begin(), transCloud.end());
	dstColorCloud.insert(dstColorCloud.end(), srcColorCloud.begin(), srcColorCloud.end());

	transCloud.clear();
	srcPointCloud.clear();
	srcColorCloud.clear();

	// time increasing with point cloud size, not linear increasing
	buildTree(dstPointCloud);
	

	// read next frame , about 350 ms
	//fileIndex++;
	//readData(fileIndex, true);
	
}

bool ModelMaker::mWriteModel(){
	char filename[30];
	char f_name[30];

	cout << "Write Model: enter model name : " ;
	cin >> filename;
	sprintf(f_name, ".\\output\\%s.txt", filename);
	ofstream out(f_name); 
	if(!out) { 
		cout << "Cannot open file.\n"; 
		return false; 
	}

	cout << "Writing output model... ";
	int nPoints = (int) dstPointCloud.size();
	out << nPoints << endl;

	for(int i = 0 ; i < nPoints ; i++){
		out << dstPointCloud[i].x << " "
			<< dstPointCloud[i].y << " "
			<< dstPointCloud[i].z << " "
			<< dstColorCloud[i][0]<< " "
			<< dstColorCloud[i][1]<< " "
			<< dstColorCloud[i][2]<< endl;
	}
	out.close(); 

	cout << "Done!" << endl;
	return true;
}

bool ModelMaker::mReadModel(){
	char filename[30];
	char f_name[30];

	cout << "Read Model : enter model name : " ;
	cin >> filename;
	sprintf(f_name, ".\\output\\%s.txt", filename);
	ifstream file(f_name); 
	if(!file) { 
		cout << "Cannot open file.\n"; 
		return false; 
	}

	cout << "Reading model... ";
	int nPoints;
	file >> nPoints;
	inPointCloud.resize(nPoints);
	inColorCloud.resize(nPoints);
	float x, y, z, r, g, b;
	float sx = 0.f, sy = 0.f, sz = 0.f;
	
	for(int i = 0 ; i < nPoints ; i++){
		file >> x >> y >> z >> r >> g >> b ;
		inPointCloud[i].x = x;
		inPointCloud[i].y = y;
		inPointCloud[i].z = z;
		inColorCloud[i][0] = r;
		inColorCloud[i][1] = g;
		inColorCloud[i][2] = b;
		sx += x;
		sy += y;
		sz += z;
		//cout << inPointCloud[i].x << " " << inPointCloud[i].y << " " << inPointCloud[i].z << endl;
	}
	inCenter.x = sx/nPoints;
	inCenter.y = sy/nPoints;
	inCenter.z = sz/nPoints;

	file.close(); 
	printf("size1 :%d  ", inPointCloud.size());
	printf("size2 :%d  \n", inColorCloud.size());

	cout << "Done! #vertex= " << nPoints << endl;
	return true;
}

void ModelMaker::alphaBlendingSwitch(){
	if(glIsEnabled(GL_BLEND)){
		glDisable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
	}else{
		glEnable(GL_BLEND);
		glDisable(GL_DEPTH_TEST);
	}
}

void ModelMaker::selectFrame(){
	printf("Select Frame - Please Enter Frame Index: ");
	scanf("%d", &fileIndex);
	readData(fileIndex, true);
}

void ModelMaker::automaticMerge(){
	
	if(doMatching)	return;
	if(amDone)		return;

	// the first time enter this function, to initialization
	if(!amInit){
		amInit = true;

		readData(amFIndex, false);
		if( (amFIndex+1) <= amList[amPIndex].second )	amFIndex++;
		else{
			if(amPIndex+1 < amList.size()){
				amPIndex++;
				amFIndex = amList[amPIndex].first;
			}
			else{
				amDone = true;
				printf("auto merge complete!!!!\n");
				return;
			}
		}
		readData(amFIndex, true);
		buildTree(dstPointCloud);

		doMatching = true;
		return;
	}

	mergeSrcDstPClouds();


	if( (amFIndex+1) <= amList[amPIndex].second )	amFIndex++;
	else{
		mtPitch  = 0.f;
		mtYaw	 = 0.f;
		mtRoll	 = 0.f;
		mtTransX = 0.f;
		mtTransZ = 0.f;
		mtTransY = 0.f;
		if(amPIndex+1 < amList.size()){
			amPIndex++;
			amFIndex = amList[amPIndex].first;
		}
		else{
			amDone = true;
			printf("auto merge complete!!!!\n");
			return;
		}
	}
	readData(amFIndex, true);
	buildTree(dstPointCloud);
	doMatching = true;
}

void ModelMaker::registerKey(int key){
	switch(key){
	case 27:	exit(0);				break;
	case 'q':	mtYaw	+= mtRStep;		break;
	case 'w':	mtYaw	-= mtRStep;		break;
	case 'a':	mtPitch  += mtRStep;	break;
	case 's':	mtPitch  -= mtRStep;	break;
	case 'z':	mtRoll	+= mtRStep;		break;
	case 'x':	mtRoll	-= mtRStep;		break;
	case 'e':	mtTransX += mtTStep;	break;
	case 'r':	mtTransX -= mtTStep;	break;
	case 'd':	mtTransY += mtTStep;	break;
	case 'f':	mtTransY -= mtTStep;	break;
	case 'c':	mtTransZ += mtTStep;	break;
	case 'v':	mtTransZ -= mtTStep;	break;
	case 't':	mtRStep *= 2;			break;
	case 'y':	mtRStep *= 0.5;			break;
	case 'g':	mtTStep *= 2;			break;
	case 'h':	mtTStep *= 0.5;			break;

	case 'n':	computeAllGradientEnergys();	break;
	/* Color Mode : Original Color / (Red/Blue) Color */
	case 'p':	colorMode = (colorMode+1)%2;	break;
	/* Show Source Point Cloud or not*/
	case 'o':	showSrc = !showSrc;				break;
	/* Show Destination Point Cloud or not*/
	case 'O':	showDst = !showDst;				break;
	/* Turn On/Off alpha blending */
	case 'i':	alphaBlendingSwitch();			break;
	/* Merge the transformed source point cloud into dst point cloud */
	case '=':	mergeSrcDstPClouds();			break;
	/* skip this src frame and jump to the next src frame */
	case '-':	readData(++fileIndex, true);	break;
	case '_':	selectFrame();					break;
	case '0':	mWriteModel();					break;
	case '9':	mReadModel();					break;
	case '8':	showReadModel = !showReadModel;	break;
	case 'm':	doMatching = true;				break;
	}
	
	isKey = true;
}