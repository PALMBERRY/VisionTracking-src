#include "LineDetect.h"
#include <highgui.h>
#include "ListFile.h"
#include <fstream>
#include "elm.h"
#include "feature.h"
#include "bwlabel.h"
//#include <engine.h>
using namespace cv;

#define ROIHEIGHT 350 //感兴趣的区域高度
#define MININF 1E-10
#define FUSE_DIST_THRES 0.5
#define FUSE_ANGLE_THRES 0.0005
#define LINE_LENGTH_THRES 150
#define SAMPLE_POINT_TO_BORDER 9
#define INITIAL_MIN_LEN 50
#define SEGMENT_2_LineThres 5

//#define MAXINTERR 255
#define ERR_BOX_WIDTH 9
#define PI 3.1415926

#define TRAIN_MODE 0
#define EXTRACT_GOOG_PAIR 0
#define SHOWLINEPAIRS_FROM_SEGMENT_2_LINE 0
#define SHOW_FITLINE_IN_FINDBESTLINE 0

extern inline string num2str(int i);

template <typename T>
inline double getInterplated(const cv::Mat &M, const double &xd, const double &yd){
	const cv::Mat_<T> M_(M);
	int x = xd; double dx = xd - x;  //每个差值
	int y = yd; double dy = yd - y;
	double dxdy = dx*dy;
	return  dxdy*M_(y + 1, x + 1) + (dy - dxdy)*M_(y + 1, x) +
		(dx - dxdy)*M_(y, x + 1) + (1 - dx - dy + dxdy)*M_(y, x);
}

inline Vec3d getInterplatedVec3b(const cv::Mat &M, const double &xd, const double &yd){
	assert(M.channels() == 3);
	const cv::Mat_<Vec3b> M_(M);
	int x = xd; double dx = xd - x;  //每个差值
	int y = yd; double dy = yd - y;
	double dxdy = dx*dy;
	return  dxdy*M_(y + 1, x + 1) + (dy - dxdy)*M_(y + 1, x) +
		(dx - dxdy)*M_(y, x + 1) + (1 - dx - dy + dxdy)*M_(y, x);
}

void nihe(double *x, double *y, int N,double &a,double &b)
{
	double sum_x_square = 0;
	double sum_x = 0;
	double sum_y = 0;
	double sum_x_y = 0;
	double x_sum_square = 0;

	for (int i = 0; i < N; i++)
	{
		sum_x_square += x[i] * x[i];
		sum_y += y[i];
		sum_x += x[i];
		sum_x_y += x[i] * y[i];
	}
	x_sum_square = sum_x*sum_x;

	a = ((sum_x_square*sum_y) - (sum_x*sum_x_y)) / (N*sum_x_square - x_sum_square);
	b = ((N*sum_x_y) - (sum_x*sum_y)) / (N*sum_x_square - x_sum_square);
}

bool extractLineFeature(double st1, double ed1, double st2, double ed2, const Mat &img, const Mat &lbpMap, Mat &feature){
	double x1 = st1; double y1 = ed1;
	double x2 = st2; double y2 = ed2;

	double len = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));

	if (len < 10)
		return false;
	double stepX = (x2 - x1) / len; double stepY = (y2 - y1) / len;


	feature.release();
	//Mat lbpMap;
	//build_lbp_map(img, lbpMap);
	Mat_<uchar> _lbpMap(lbpMap);

	const int singleChannelLength = 10;
	const int lbpLen = 10;
	const int allFeatureLen = 3 * singleChannelLength + lbpLen;
	double featureD[allFeatureLen];
	memset(featureD, 0, sizeof(double)* allFeatureLen);

	float stepValve = 255.5 / singleChannelLength;  //calculate the step of every channel

	int boderWidth = ERR_BOX_WIDTH / 2;
	
	double curX = x1;
	double curY = y1;

	double stepXL = -stepY; double stepYL = stepX;
	double err = 0;
	double binErr = 0;
	double usedPointNum = 0;

	vector<Vec3d> sample1;
	vector<Vec3d> sample2;
	while ((stepX < 0) == (curX > x2) && (stepY < 0) == (curY > y2)){
		curX += stepX; curY += stepY;
		if (curX<SAMPLE_POINT_TO_BORDER || curX>img.cols - SAMPLE_POINT_TO_BORDER
			|| curY<SAMPLE_POINT_TO_BORDER || curY>img.rows - SAMPLE_POINT_TO_BORDER)
			continue;

		double tempErrGray = 0;


		for (int k = 1; k < boderWidth; ++k){
			double x1 = curX - k* stepXL;
			double y1 = curY - k*stepYL;
			double x2 = curX + k* stepXL;
			double y2 = curY + k*stepYL;

			//cout << int(x1 + 0.5) << " " << int(y1 + 0.5) << endl;
			//cout << int(x2 + 0.5) << " " << int(y2 + 0.5) << endl;
			if (lbpLen){
				++featureD[_lbpMap(int(y1 + 0.5), int(x1 + 0.5)) + 3 * singleChannelLength];
				++featureD[_lbpMap(int(y2 + 0.5), int(x2 + 0.5)) + 3 * singleChannelLength];
			}

			if (singleChannelLength){
				Vec3d p1 = getInterplatedVec3b(img, x1, y1);
				Vec3d p2 = getInterplatedVec3b(img, x2, y2);
				p1[0] *= 255.0 / 180;
				p2[0] *= 255.0 / 180;

				p1 /= stepValve; p2 /= stepValve;
				usedPointNum += 2;
				++featureD[int(p1[0])]; ++featureD[int(p1[1] + singleChannelLength)]; ++featureD[int(p1[2] + 2 * singleChannelLength)];
				++featureD[int(p2[0])]; ++featureD[int(p2[1] + singleChannelLength)]; ++featureD[int(p2[2] + 2 * singleChannelLength)];
			}
		}
	}

	if (usedPointNum < 1)
		return false;
	Mat(1, allFeatureLen, CV_64FC1, (void *)featureD).copyTo(feature);
	feature /= usedPointNum;
	return true;
}
void edgeEnhance(cv::Mat& srcImg, cv::Mat& dstImg)
{

	std::vector<cv::Mat> rgb;

	if (srcImg.channels() == 3)        // rgb image  
	{
		cv::split(srcImg, rgb);
	}
	else if (srcImg.channels() == 1)   // gray image  
	{
		rgb.push_back(srcImg);
	}

	// 分别对R、G、B三个通道进行边缘增强  
	for (size_t i = 0; i < rgb.size(); i++)
	{
		cv::Mat sharpMat8U;
		cv::Mat sharpMat;
		cv::Mat blurMat;

		// 高斯平滑  
		cv::GaussianBlur(rgb[i], blurMat, cv::Size(5, 5), 0, 0);

		// 计算拉普拉斯  
		cv::Laplacian(blurMat, sharpMat, CV_16S);

		// 转换类型  
		sharpMat.convertTo(sharpMat8U, CV_8U);
		cv::add(rgb[i], sharpMat8U, rgb[i]);
	}


	cv::merge(rgb, dstImg);
}

void sharpen(const Mat& img, Mat& result)
{
	result.create(img.size(), img.type());
	//处理边界内部的像素点, 图像最外围的像素点应该额外处理
	for (int row = 1; row < img.rows - 1; row++)
	{
		//前一行像素点
		const uchar* previous = img.ptr<const uchar>(row - 1);
		//待处理的当前行
		const uchar* current = img.ptr<const uchar>(row);
		//下一行
		const uchar* next = img.ptr<const uchar>(row + 1);
		uchar *output = result.ptr<uchar>(row);
		int ch = img.channels();
		int starts = ch;
		int ends = (img.cols - 1) * ch;
		for (int col = starts; col < ends; col++)
		{
			//输出图像的遍历指针与当前行的指针同步递增, 以每行的每一个像素点的每一个通道值为一个递增量, 因为要考虑到图像的通道数
			*output++ = saturate_cast<uchar>(5 * current[col] - current[col - ch] - current[col + ch] - previous[col] - next[col]);
		}
	} //end loop
	//处理边界, 外围像素点设为 0
}

void enhanceRGB(cv::Mat& srcImg, cv::Mat& dstImg){
	Mat imageYCrCb;
	cvtColor(srcImg, srcImg, CV_RGB2YCrCb);
	vector<Mat> singleC;
	split(srcImg, singleC);
	equalizeHist(singleC[0], singleC[0]);
	merge(singleC, dstImg);
	cvtColor(dstImg, dstImg, CV_YCrCb2RGB);
}

LineDetector::LineDetector(){
	loadModel();
	loadGauss();
	trainSampleIndex = 0;
	lines.clear();
	lsd = createLineSegmentDetectorPtr(1, 0.8, 0.6, 2, 22.5, 0, 0, 1024);
	rolLabel = -1; colLabel = -1;
}

void LineDetector::updatePara(){
	cameraMatrix = Mat(3, 3, CV_64FC1, (void *)cameraPara);
	distMatrix = Mat(1, 4, CV_64FC1, (void *)distorPara);

	double q0 = para[0], q1 = para[1], q2 = para[2], q3 = para[3], h = para[4];
	double R[9] = {
		2 * q0*q0 + 2 * q1*q1 - 1, 2 * q1*q2 - 2 * q0*q3, 2 * q1*q3 + 2 * q0*q2,
		2 * q1*q2 + 2 * q0*q3, 2 * q0*q0 + 2 * q2*q2 - 1, 2 * q2*q3 - 2 * q0*q1,
		2 * q1*q3 - 2 * q0*q2, 2 * q2*q3 + 2 * q0*q1, 2 * q0*q0 + 2 * q3*q3 - 1
	};

	Mat R_M(3, 3, CV_64F, (void *)R);
	warp3D = R_M*cameraMatrix.inv();
	warp3D.row(0) *= -h;
	warp3D.row(1) *= -h;

	warp3DInv = warp3D.inv();
	//warp3DBias = warp3D;
	//warp3DBias.row(0) += 500;
	//warp3DBias.row(1) -= 1000;
}

void LineDetector::setCamera(const double *K, const double *distor,const int _imageHeight,const int _imageWidth){
	roi = Rect(0, _imageHeight - ROIHEIGHT, _imageWidth, ROIHEIGHT);
	imageHeight = _imageHeight;
	imageWidth = _imageWidth;
	for (int i = 0; i < 9; ++i)
		cameraPara[i] = K[i];
	for (int i = 0; i < 4; ++i)
		distorPara[i] = distor[i];
	updatePara();
}

void LineDetector::setExtra(const double *RT){
	for (int i = 0; i < 5; ++i)
		para[i] = RT[i];
	updatePara();
}

void LineDetector::getGoodLinesPair(vector<LineCluter> lineCluters){
	Mat drawCur2;
	roiImageC.copyTo(drawCur2);
	/*if (!usefulCluters.size()){
	imshow("EVENTLINE", drawCur2);
	waitKey(30);
	return;
	}*/
	for (int i = 0; i < lineCluters.size(); ++i){
		if (lineCluters[i].lenAll<LINE_LENGTH_THRES)
			continue;
		vector<Point2d> s;
		findPt(lineCluters[i].meanA, lineCluters[i].meanB, lineCluters[i].meanC, s);
		line(drawCur2, s[0], s[1], Scalar(0, 0, 255), 1);
		//cout << lineCluters[i].angle << endl;
	}

	imshow("ALLLINE", drawCur2);
	waitKey(30);
	cout << "是否有好的直线对：" << endl;
	int a = waitKey(0);
	destroyWindow("ALLLINE");
	
	if (a==48)
		return;
	
	ofstream fp("goodPair.txt", ios::app);
	
	for (int i = 0; i < lineCluters.size(); ++i){
		if (lineCluters[i].lenAll < LINE_LENGTH_THRES)
			continue;
		for (int j = i + 1; j < lineCluters.size(); ++j){
			if (lineCluters[j].lenAll < LINE_LENGTH_THRES)
				continue;
			
			Mat drawCur2;
			roiImageC.copyTo(drawCur2);

			vector<Point2d> s;
			findPt(lineCluters[i].meanA, lineCluters[i].meanB, lineCluters[i].meanC, s);
			line(drawCur2, s[0], s[1], Scalar(0, 0, 255), 1);

			vector<Point2d> s2;
			findPt(lineCluters[j].meanA, lineCluters[j].meanB, lineCluters[j].meanC, s2);
			line(drawCur2, s2[0], s2[1], Scalar(0, 255, 0), 1);

			imshow("Pair", drawCur2);
			waitKey(30);

			cout << "good Pair？" << endl;
			
			int a = waitKey(0);
			if (a==48)
				continue;

			fp << s[0].x << "\t" << s[0].y + imageHeight - ROIHEIGHT << "\t" << s[1].x << "\t" << s[1].y + imageHeight- ROIHEIGHT
				<< "\t" << s2[0].x << "\t" << s2[0].y + imageHeight - ROIHEIGHT << "\t" << s2[1].x << "\t" << s2[1].y + imageHeight-ROIHEIGHT << endl;
			destroyWindow("Pair");
		}
	}
	fp.close();
}

void LineDetector::clearAll(){
	angle = -1;//输出的数据初始化
	leftNearestDis = rightNearestDis = -1;
	leftNearestIndex = -1;
	rightNearestIndex = -1;

	lines.clear();
	lineCluters.clear();
	lineGetInf.clear();
	usefulCluters.clear();
}

void LineDetector::processImage(const Mat &inputImageOrg, bool labelDetectFlag, Result &detectResult, double interestAngle,bool trainMode, int imageIndex){
	detectResult.reset();
	clearAll();

	if (interestAngle > 0){
		angleMin = 90 - interestAngle;
		angleMax = 90 + interestAngle;
	}
	else
	{
		angleMin = 0;
		angleMax = 180;
	}

	bool ShowFlag = false;	
	undistort(inputImageOrg, inputImage, cameraMatrix, distMatrix);

	Mat  roiImageG, roiImageH;
	roiImageC = inputImage(roi);// 

	//enhanceRGB (roiImageC, roiImageC);
	/*Mat sharpened;
	sharpen(roiImageC, sharpened);
	roiImageC = sharpened;*/
	

	//medianBlur(roiImageC, roiImageC, 3);
	cvtColor(roiImageC, roiImageG, CV_RGB2GRAY);
	cvtColor(roiImageC, roiImageH, CV_RGB2HSV); //得到各个色彩空间的图像
	
	roiImageC.copyTo(outPutImg);
	vector<Mat> splited;
	split(roiImageH, splited);

	lsd->detect(splited[2], lines); //splited[1]
	if (ShowFlag){
		Mat drawCur;
		roiImageC.copyTo(drawCur);
		lsd->drawSegments(drawCur, lines);
		imshow("SegmentDetect", drawCur);
		waitKey(30);
	}
	//cout << "线条总数："<<lines.size() << endl;
	//if (lines.size() > 400) //若检测到的线条总数大于400，则当前图像不再处理
		//return;

	//showLine(lines);
	lineClutersIni();
	fuseCluter();
	if (ShowFlag)
		showLineCluter(lineCluters, "AfterFuse");

	if (trainMode)
	if (imageIndex < 0)
	{
		cout << "训练模式下必须输入图像标号并保存图像！" << endl;
		return;
	}
	else{
		gaussTrainSampleExtract(imageIndex);
		return;
	}

	//showLineCluter(lineCluters);
	findBestLine();
	if (ShowFlag)
		showLineCluter(usefulCluters, "AfterSelect");
	if (EXTRACT_GOOG_PAIR){
		getGoodLinesPair(usefulCluters);
		return;
	}
	//showLine(usefulCluters);
	calAngleDist();
	if (ShowFlag)
		showLineCluter(usefulCluters, "AfterCalAngle");
	drawLine();

	detectResult.angle = angle;
	detectResult.leftDistance = leftNearestDis;
	detectResult.rightDistance = rightNearestDis;

	if (angle<0 || (!labelDetectFlag))
		return;

	if (!boxDetect())
		detectResult.labelValid = false;
	else
	{
		detectResult.labelValid = true;
		detectResult.labelXpos = labelXPos;
		detectResult.labelYpos = labelYPos;
	}
	/*MatIterator_<Vec3b> ItInputImage = inputImage.begin<Vec3b>()+startRow*inputImage.cols; //初始遍历点
	int k = startRow*inputImage.cols;
	for (; ItInputImage != inputImage.end<Vec3b>(); ++ItInputImage,++k){
		double x, y;
		getPoint3D(k%inputImage.cols, k / inputImage.cols, x, y);
		x += biasX+0.5;
		y += biasY+0.5;
		x /= ratio;
		y /= ratio;
		if (x<0 || x>=warpedImage.cols||y<0||y>=warpedImage.rows)
			continue;
		warpedImage(y, x) = *ItInputImage;
	}*/
	//imshow("warpedImage", warpedImage);
	//waitKey(0);
}

void LineDetector::showLineCluter(const vector<LineCluter> &cluters,const std::string windowName){
	Mat drawCur;
	roiImageC.copyTo(drawCur);
	for (int i = 0; i < cluters.size(); ++i){
		if (cluters[i].lenAll<LINE_LENGTH_THRES)
			continue;
		vector<Point2d> pt;
		findPt(cluters[i].meanA, cluters[i].meanB, cluters[i].meanC, pt);
		line(drawCur, pt[0], pt[1], Scalar(0, 0, 255), 1);
	}
	imshow(windowName.c_str(), drawCur);
	waitKey(30);
}

void LineDetector::showLine(const vector<Vec4i> &interestLine){
	Mat drawCur;
	roiImageC.copyTo(drawCur);
	lsd->drawSegments(drawCur, interestLine);
	imshow("AllLine", drawCur);
	waitKey(30);
}

void LineDetector::lineClutersIni(){
	lineCluters.resize(lines.size());
	lineGetInf.resize(lines.size());
	lengthAll = 0;
	for (int i = 0; i < lines.size(); ++i){
		double x1 = lines[i](0), y1 = lines[i](1), x2 = lines[i](2), y2 = lines[i](3);

		//int x1 = lines[i](0), y1 = lines[i](1), x2 = lines[i](2), y2 = lines[i](3);*/
		float length = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)); //求直线长度
		lengthAll += length;
		lineCluters[i].lenAll = lineGetInf[i].len = length;
		lineGetInf[i].line = lines[i];

		lineCluters[i].indexOfLine.push_back(i);

		double A = y2 - y1; double B = x1 - x2; double C = x2*y1 - x1*y2;
		
		if (A < 0)
		{
			A = -A;
			B = -B;
			C = -C;
		}
		double sAll = sqrt(A*A + B*B + C*C);
		A /= sAll;
		B /= sAll;
		C /= sAll;

		lineCluters[i].meanA = lineGetInf[i].A = A;
		lineCluters[i].meanB = lineGetInf[i].B = B;
		lineCluters[i].meanC = lineGetInf[i].C = C;
	}
	return;
}

inline void LineDetector::drawLine(){
	for (int i = 0; i < usefulCluters.size(); ++i){
		if (i==leftNearestIndex||i==rightNearestIndex)
			continue;
		line(outPutImg, usefulCluters[i].bottomPixelLoc, usefulCluters[i].upPixelLoc, Scalar(0, 0, 255), 3);
	}
	if (leftNearestIndex>=0)
		line(outPutImg, usefulCluters[leftNearestIndex].bottomPixelLoc, usefulCluters[leftNearestIndex].upPixelLoc, Scalar(0, 255, 0), 3);
	if (rightNearestIndex>=0)
		line(outPutImg, usefulCluters[rightNearestIndex].bottomPixelLoc, usefulCluters[rightNearestIndex].upPixelLoc, Scalar(0, 255, 0), 3);
}

void LineDetector::fuseCluter(){
	bool chaneFlag = true;
	while (chaneFlag){
		chaneFlag = false;
		sort(lineCluters.begin(), lineCluters.end());
		if (lineCluters.size() > 400)
			lineCluters.erase(lineCluters.begin() + 399, lineCluters.end());
		int index1 = 0; int index2 = 0;
		while (index1 < lineCluters.size()) //待融合直线
		{
			if (lineCluters[index1].lenAll < INITIAL_MIN_LEN){
				++index1;
				continue;
			}

			index2 = index1 + 1; 
			double A1 = lineCluters[index1].meanA; double B1 = lineCluters[index1].meanB;
			double C1 = lineCluters[index1].meanC;

			while (index2 < lineCluters.size())
			{
				//double errA = lineCluters[index1].meanA - lineCluters[index2].meanA;
				//double errB = lineCluters[index1].meanB - lineCluters[index2].meanB;
				//首先显示出两条直线

				if (lineCluters[index2].indexOfLine.size()>1){ //如果index2这个cluter里有多条线段
					double A2 = lineCluters[index2].meanA; double B2 = lineCluters[index2].meanB;

					double errC = abs(abs(C1) / sqrt(A1*A1 + B1*B1) - abs(lineCluters[index2].meanC) / sqrt(A2*A2 + B2*B2));
					//double errAll = abs(errA) + abs(errB) + abs(errC);
					double errAngle = 1-abs(A1*A2 + B1*B2) / (sqrt(A1*A1 + B1*B1)*sqrt(A2*A2 + B2*B2));
					if (SHOWLINEPAIRS_FROM_SEGMENT_2_LINE)
						cout << "1：" << errC << " " << errAngle << endl;
					if (errC>FUSE_DIST_THRES||errAngle>FUSE_ANGLE_THRES){
						++index2;
						continue;
					}
				}
				else{   //如果只有一条线段，则对比端点到直线的距离

					int curLineIndex = lineCluters[index2].indexOfLine[0];
					double x1 = lines[curLineIndex](0), y1 = lines[curLineIndex](1), x2 = lines[curLineIndex](2), y2 = lines[curLineIndex](3);
					double dist1 = abs(A1*x1 + B1*y1 + C1) / sqrt(A1*A1 + B1*B1);
					double dist2 = abs(A1*x2 + B1*y2 + C1) / sqrt(A1*A1 + B1*B1);
					if (SHOWLINEPAIRS_FROM_SEGMENT_2_LINE)
						cout << "2：" << dist1 + dist2 << endl;
					if (dist1 + dist2 > SEGMENT_2_LineThres){
						++index2;
						continue;
					}
				}

				if (SHOWLINEPAIRS_FROM_SEGMENT_2_LINE){
					Mat drawCur;
					roiImageC.copyTo(drawCur);
					vector<Vec4i> tempLine;
					for (int i = 0; i < lineCluters[index1].indexOfLine.size(); ++i)
						tempLine.push_back(lines[lineCluters[index1].indexOfLine[i]]);
					for (int i = 0; i < lineCluters[index2].indexOfLine.size(); ++i)
						tempLine.push_back(lines[lineCluters[index2].indexOfLine[i]]);
					lsd->drawSegments(drawCur, tempLine);
					imshow("tempLine", drawCur);
					waitKey(30);
				}

				
				//if (index1==0&&index2==48)
				//	cout << index1 << " " << index2 << endl;
				if (!sampleLineToFit(index1, index2, lineCluters[index1].meanA, lineCluters[index1].meanB
					, lineCluters[index1].meanC))
				{
					++index2;
					continue;
				}
				chaneFlag = true; //

				/*lineCluters[index1].meanA = (lineCluters[index1].meanA*lineCluters[index1].lenAll + 
					lineCluters[index2].meanA*lineCluters[index2].lenAll);
				lineCluters[index1].meanB = (lineCluters[index1].meanB*lineCluters[index1].lenAll +
					lineCluters[index2].meanB*lineCluters[index2].lenAll);
				lineCluters[index1].meanC = (lineCluters[index1].meanC*lineCluters[index1].lenAll +
					lineCluters[index2].meanC*lineCluters[index2].lenAll); //计算两者的均值*/

				lineCluters[index1].lenAll += lineCluters[index2].lenAll;

				for (int i = 0; i < lineCluters[index2].indexOfLine.size(); ++i)
					lineCluters[index1].indexOfLine.push_back(lineCluters[index2].indexOfLine[i]);
				/*double normlizer = sqrt(lineCluters[index1].meanA*lineCluters[index1].meanA + lineCluters[index1].meanB*lineCluters[index1].meanB
					+ lineCluters[index1].meanC*lineCluters[index1].meanC);

				lineCluters[index1].meanA /= normlizer;
				lineCluters[index1].meanB /= normlizer;
				lineCluters[index1].meanC /= normlizer;*/
				
				lineCluters[index2] = lineCluters.back(); 
				lineCluters.pop_back();
			}

			++index1;
		}
	}
	sort(lineCluters.begin(), lineCluters.end());
	if (SHOWLINEPAIRS_FROM_SEGMENT_2_LINE)
		for (int i = 0; i < lineCluters.size(); ++i){
			Mat drawCur;

			roiImageC.copyTo(drawCur);

			Mat drawCur2;
			roiImageC.copyTo(drawCur2);

			vector<Vec4i> tempLine;
			for (int k = 0; k < lineCluters[i].indexOfLine.size(); ++k)
				tempLine.push_back(lines[lineCluters[i].indexOfLine[k]]);
			lsd->drawSegments(drawCur, tempLine);
			vector<Point2d> s;

			findPt(lineCluters[i].meanA, lineCluters[i].meanB, lineCluters[i].meanC, s);
			line(drawCur2, s[0], s[1], Scalar(255, 255, 255), 1);
			imshow("fitLine", drawCur2);
			waitKey(30);

			imshow("LSDDetectLine", drawCur);
			waitKey(0);
		}
}

/*inline void extractFeatureOfLine(const vector<Vec3d> &sapmle, Mat &feature,double &sumMean){
	if (sapmle.size() < 1)
		return;

	double mean1 = 0,mean2=0,mean3=0;
	for (int i = 0; i < sapmle.size(); ++i){
		mean1 += sapmle[i](0);
		mean2 += sapmle[i](1);
		mean3 += sapmle[i](2);
	}

	mean1 /= sapmle.size(); //求均值
	mean2 /= sapmle.size();
	mean3 /= sapmle.size();

	sumMean = 0;
	sumMean += mean1 + mean2 + mean3;

	double var1 = 0, var2 = 0, var3 = 0;
	for (int i = 0; i < sapmle.size(); ++i){
		var1 += (sapmle[i](0) - mean1)*(sapmle[i](0) - mean1);
		var2 += (sapmle[i](1) - mean2)*(sapmle[i](1) - mean2);
		var3 += (sapmle[i](2) - mean3)*(sapmle[i](2) - mean3);
	}

	var1 /= sapmle.size();
	var2 /= sapmle.size();
	var3 /= sapmle.size();
	double Mat7[7] = { sapmle.size(), mean1, mean2, mean3, var1, var2, var3 };
	Mat(7, 1, CV_64FC1, (void *)Mat7).copyTo(feature);
}*/
void LineDetector::gaussTrainSampleExtract(int picIndex){

	//double tempLenAll = 0;
	string windowName("TrainSampleExtract");
	namedWindow(windowName.c_str());
	for (int i = 0; i < lineCluters.size(); ++i){
		if (lineCluters[i].lenAll<LINE_LENGTH_THRES) //(!SHOW_FITLINE_IN_FINDBESTLINE)&&
			continue;
		vector<Point2d> pt;
		findPt(lineCluters[i].meanA, lineCluters[i].meanB, lineCluters[i].meanC, pt);

		Mat drawCur2;
		roiImageC.copyTo(drawCur2);
		line(drawCur2, pt[0], pt[1], Scalar(0, 255, 0), 1);
		imshow(windowName.c_str(), drawCur2);
		cout << "当前直线有效？" << endl;
		int a = waitKey(0);

		//cout << featureAll << endl;
		string file("s");

		++trainSampleIndex;
		ofstream fs("SamplesLines.txt", ios::app);
		if (a == 48){
			fs << picIndex << " " << -1 << " " << pt[0].x << " " << pt[0].y << " " << pt[1].x << " " << pt[1].y << endl;
			fs.close();
			cout << "---得到新的训练负样本！" << endl;
			continue;
		}

		fs << picIndex << " " << +1 << " " << pt[0].x << " " << pt[0].y << " " << pt[1].x << " " << pt[1].y << endl;
		fs.close();
		cout << "---得到新的训练正样本！" << endl;
	}
	destroyWindow(windowName.c_str());
}


void LineDetector::findBestLine(){
	Mat lbpMap;
	build_lbp_map(roiImageC, lbpMap);

	Mat roiHSV;
	cvtColor(roiImageC, roiHSV, CV_RGB2HSV);

	//cvtColor(roiImageC, roiImageH, CV_RGB2HSV);
	int bestIndex = 0;
	
	//double tempLenAll = 0;
	for (int i = 0; i < lineCluters.size(); ++i){

		if (lineCluters[i].lenAll<LINE_LENGTH_THRES) //(!SHOW_FITLINE_IN_FINDBESTLINE)&&
			continue;

		vector<Point2d> pt;
		findPt(lineCluters[i].meanA, lineCluters[i].meanB, lineCluters[i].meanC, pt);
		if (pt[0].y>pt[1].y)
		{
			lineCluters[i].bottomPixelLoc = pt[0];
			lineCluters[i].upPixelLoc = pt[1];
		}
		else{
			lineCluters[i].bottomPixelLoc = pt[1];
			lineCluters[i].upPixelLoc = pt[0];
		}
		Mat feature;
		bool sucessFlag = extractLineFeature(pt[0].x, pt[0].y, pt[1].x, pt[1].y, roiHSV, lbpMap, feature);
		//cout << feature << endl;
		if (!sucessFlag)
			continue;
	
		Mat score;
		elm_predict(feature, InputWeight, Bias, OutputWeight, score);
		
		if (SHOW_FITLINE_IN_FINDBESTLINE){
			cout << usefulCluters.size() << endl;
			cout << "sss：" << score.col(0) << endl;
			Mat drawCur2;
			roiImageC.copyTo(drawCur2);
			vector<Point2d> s;
			findPt(lineCluters[i].meanA, lineCluters[i].meanB, lineCluters[i].meanC, s);
			line(drawCur2, s[0], s[1], Scalar(0, 255, 0), 1);
			imshow("fitLine", drawCur2);
			waitKey(0);
		}
		//计算角度，角度需要满足一定范围
		if (score.at<double>(0, 0) < 0.1)
			continue;

		lineCluters[i].score = score.at<double>(0, 0);
		usefulCluters.push_back(lineCluters[i]);
	}
	sort(usefulCluters.begin(), usefulCluters.end());
}



void LineDetector::findPt(const double &A, const double &B, const double &C,vector<Point2d> &pt){
	if (abs(A) > MININF&& -C / A >= 0 && -C / A < roiImageC.cols)
		pt.push_back(Point2d(-C / A, 0));

	if (abs(A)>MININF && -(B*roiImageC.rows + C) / A>=0 && -(B*roiImageC.rows + C) / A<roiImageC.cols)
		pt.push_back(Point2d(-(B*roiImageC.rows + C) / A, roiImageC.rows));

	if (pt.size() == 2)
		return;

	if (abs(B)>MININF&& -C / B>=0 && -C / B<roiImageC.rows)
		pt.push_back(Point2d(0, -C/B));

	if (pt.size() == 2)
		return;

	if (abs(B)>MININF&& -(A*roiImageC.cols + C) / B >= 0 && -(A*roiImageC.cols + C) / B < roiImageC.rows)
		pt.push_back(Point2d(roiImageC.cols, -(A*roiImageC.cols + C) / B));

}

bool LineDetector::sampleLineToFit(int index1, int index2, double &A, double &B, double &C){
	int lenAll = 2*lineCluters[index1].lenAll;
	lenAll += 2*lineCluters[index2].lenAll;
	double *x = new double[lenAll];
	double *y = new double[lenAll];

	int curLenIndex = 0;
	double boderLen = 1;
	double sumX = 0, sumY = 0;
	double sumLen = 0;
	for (int i = 0; i < lineCluters[index1].indexOfLine.size(); ++i){ //对index1采样
		int curLineIndex = lineCluters[index1].indexOfLine[i];
		double x1 = lines[curLineIndex](0), y1 = lines[curLineIndex](1), x2 = lines[curLineIndex](2), y2 = lines[curLineIndex](3);
		double len = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
		sumLen += len;
		double stepX = (x2 - x1) / len; double stepY = (y2 - y1) / len;
		double curX = x1;
		double curY = y1;
		while (((stepX < 0) == (curX > x2)) && (stepY < 0) == (curY > y2)){
			x[curLenIndex] = curX;
			y[curLenIndex] = curY;
			sumX += stepX;
			sumY += stepY;
			curX += stepX; curY += stepY;
			++curLenIndex;
		}
	}

	for (int i = 0; i < lineCluters[index2].indexOfLine.size(); ++i){
		int curLineIndex = lineCluters[index2].indexOfLine[i];
		double x1 = lines[curLineIndex](0), y1 = lines[curLineIndex](1), x2 = lines[curLineIndex](2), y2 = lines[curLineIndex](3);
		double len = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
		double stepX = (x2 - x1) / len; double stepY = (y2 - y1) / len;
		double curX = x1;
		double curY = y1;
		while (((stepX < 0) == (curX > x2)) && (stepY < 0) == (curY > y2)){
			x[curLenIndex] = curX;
			y[curLenIndex] = curY;
			sumX += stepX;
			sumY += stepY;
			curX += stepX; curY += stepY;
			++curLenIndex;
		}
	}
	if (curLenIndex<lineCluters[index1].lenAll){
		delete[] x;
		delete[] y;
		return false;
	}
	double a, b;
	if (abs(sumX) > abs(sumY)){
		nihe(x, y, curLenIndex, a, b);
		A = b; B = -1; C = a;
		double s = sqrt(A*A + B*B + C*C);
		A /= s; B /= s; C /= s;
	}
	else{
		nihe(y, x, curLenIndex, a, b);
		A = -1; B = b; C = a;
		double s = sqrt(A*A + B*B + C*C);
		A /= s; B /= s; C /= s;
	}
	delete[] x;
	delete[] y;
	return true;
}

bool LineDetector::loadModel(){ //从文件中读入高斯模型
	FileStorage fs2("elm.xml", FileStorage::READ); //便于多文件计算gauss
	if (!fs2.isOpened()){
		cout << "打开文件错误" << endl;
		return false;
	}
	fs2["InputWeight"]>> InputWeight;
	fs2["Bias"] >> Bias;
	fs2["OutputWeight"] >> OutputWeight;
	fs2.release();
	return true;
}

bool LineDetector::loadGauss(){ //从文件中读入高斯模型
	string folder("trained//*.xml");
	FileLister fileLister;
	vector<string> fileName = fileLister.get_filelist(folder.c_str());
	//for (int i = 0; i<fileName.size(); ++i){

	FileStorage fs2("purplelabel.xml", FileStorage::READ); //便于多文件计算gauss
	if (!fs2.isOpened()){
		cout << "打开文件错误" << endl;
		return false;
	}

	cluter cur;
	fs2["cov"] >> cur.cov;
	fs2["mu"] >> cur.center;
	fs2["covinv"] >> cur.covinv;
	fs2["sampleNum"] >> cur.sampleNum;
	labelGMM = cur;
	//}
	return true;
}
void LineDetector::calAngleDist(){
	int bestScoreIndex = 0;
	for (int i = 0; i < usefulCluters.size(); ){
		double A = usefulCluters[i].meanA;
		double B = usefulCluters[i].meanB;
		double C = usefulCluters[i].meanC;
		Point2d ptBottom = usefulCluters[i].bottomPixelLoc;
		Point2d ptUp = usefulCluters[i].upPixelLoc;
		double uMin, vMin, uMax, vMax;
		
		vMin = ptUp.y;
		uMin = ptUp.x;
		vMax = ptBottom.y;
		uMax = ptBottom.x;

		double len = sqrt((uMin - uMax)*(uMin - uMax) + (vMin - vMax)*(vMin - vMax));
		double stepx = (uMax - uMin)/len;
		double stepy = (vMax - vMin) / len;
		double pClose[2] = { uMax, vMax + imageHeight-ROIHEIGHT};
		double pFar[2] = { uMax - stepx, vMax - stepy + imageHeight-ROIHEIGHT};

		getPoint3D(pClose[0], pClose[1], pClose[0], pClose[1]);
		getPoint3D(pFar[0], pFar[1], pFar[0], pFar[1]);

		if (pClose[1] < 0 || pClose[1]>5000){
			usefulCluters[i] = usefulCluters.back();
			usefulCluters.pop_back();
			continue;
		}
		
		double vec[2] = { pFar[0] - pClose[0], pFar[1] - pClose[1] };
		double lenAll = sqrt(vec[0] * vec[0] + vec[1] * vec[1]);

		double tempA = vec[1], tempB = -vec[0], tempC = vec[0] * pClose[1]-vec[1]*pClose[0];
		double tempNorm = sqrt(tempA*tempA + tempB*tempB + tempC*tempC);

		usefulCluters[i].A_3d = tempA / tempNorm;
		usefulCluters[i].B_3d = tempB / tempNorm;
		usefulCluters[i].C_3d = tempC / tempNorm;

		double curAng = acos(vec[0] / lenAll);
		usefulCluters[i].angle = curAng / PI * 180;

		if (usefulCluters[i].angle<angleMin || usefulCluters[i].angle>angleMax){
			usefulCluters[i] = usefulCluters.back();
			usefulCluters.pop_back();
			continue;
		}

		usefulCluters[i].score *= sin(curAng);

		
		if (0){ //显示当前线以及角度
			cout << usefulCluters[i].angle << endl;
			Mat drawCur2;
			roiImageC.copyTo(drawCur2);
			line(drawCur2, usefulCluters[i].bottomPixelLoc, usefulCluters[i].upPixelLoc, Scalar(255, 255, 255), 1);
			imshow("sssss", drawCur2);
			waitKey(0);
		}
		if (usefulCluters[bestScoreIndex].score < usefulCluters[i].score)
			bestScoreIndex = i;
		++i;
	}

	double allScore = 0;
	angle = 0;
	if (usefulCluters.size() == 0){
		angle = -1;
		return;
	}

	leftNearestDis = rightNearestDis = 100000000000;

	LineCluter tempBestScore = usefulCluters[bestScoreIndex];
	usefulCluters[bestScoreIndex] = usefulCluters[0];
	usefulCluters[0] = tempBestScore;
	bestScoreIndex = 0;

	for (int i = 0; i < usefulCluters.size(); ){
		if (abs(usefulCluters[i].angle - usefulCluters[bestScoreIndex].angle)>10){
			usefulCluters[i] = usefulCluters.back();
			usefulCluters.pop_back();
			continue;
		}
		//cout << usefulCluters[i].angle << endl;
		if (0){
			Mat outPutImg;
			roiImageC.copyTo(outPutImg);
			line(outPutImg, usefulCluters[i].bottomPixelLoc, usefulCluters[i].upPixelLoc, Scalar(0, 0, 255), 1);
			imshow("tempLine", outPutImg);
			waitKey(0);
		}
		double tempA = usefulCluters[i].A_3d, tempB = usefulCluters[i].B_3d, tempC = usefulCluters[i].C_3d;
		double tempDis = abs(tempC) / sqrt(tempA*tempA + tempB*tempB);
		if (usefulCluters[i].bottomPixelLoc.x < cameraPara[2]){
			if (tempDis < leftNearestDis)
			{
				leftNearestDis = tempDis;
				leftNearestIndex = i;
			}
		}
		else{
			if (tempDis < rightNearestDis)
			{
				rightNearestDis = tempDis;
				rightNearestIndex = i;
			}
		}

		allScore += usefulCluters[i].score;
		angle += usefulCluters[i].angle * usefulCluters[i].score;

		++i;
	}
	angle /= allScore+MININF;
	if (leftNearestIndex < 0)
		leftNearestDis = -1;

	if (rightNearestIndex < 0)
		rightNearestDis = -1;
}

struct PointCluter{
	double meanX;
	double meanY;
	double refinedX;
	double refinedY;
	double score;
	double A;
	double B;
	double C;
	int pointNum;
	vector<Point2f> allPoint;
	RotatedRect boxRect;
	PointCluter() :meanX(0), meanY(0), pointNum(0),refinedX(-1),refinedY(-1),score(0){}
	void normlize(){
		meanY /= pointNum;
		meanX /= pointNum;
	}
};

struct color
{
	int threshold_value1;
	int threshold_value2;
	int threshold_value3;
	int threshold_value4;
	int threshold_value5;
	int threshold_value6;
};

void getThresFromCluter(const cluter &curCluter, color &curColor){
	double h = curCluter.center.at<double>(0, 0);
	double s = curCluter.center.at<double>(0, 1);
	double v = curCluter.center.at<double>(0, 2);
	double sigmaH = sqrt(curCluter.cov.at<double>(0, 0));
	double sigmaS = sqrt(curCluter.cov.at<double>(1, 1));
	double sigmaV = sqrt(curCluter.cov.at<double>(2, 2));
	double ratio = 2.5;
	curColor.threshold_value1 = h - ratio*sigmaH;
	curColor.threshold_value2 = h + ratio*sigmaH;
	curColor.threshold_value3 = s - ratio*sigmaS;
	curColor.threshold_value4 = s + ratio*sigmaS;
	curColor.threshold_value5 = v - ratio*sigmaV;
	curColor.threshold_value6 = v + ratio*sigmaV;
}

bool LineDetector::boxDetect(){ //检测贴在直线上的标签，首先该标签必须位于直线上
	//得到特定区域，只对最大距离内的点进行检测
	color interestColor;
	getThresFromCluter(labelGMM, interestColor);
	labelYPos = 3500;
	double boxLen = 150;
	double boxWidth = 50;

	double maxDistDetect = 2000;
	double minDistDetect = 150;

	double u, v, x, y, biasX, biasY, startRow;
	getPointInImageFrom3D(0, maxDistDetect, u, v);
	startRow = v;
	getPoint3D(0, v, biasX, y);
	biasX = -biasX;
	getPoint3D(inputImage.cols / 2, inputImage.rows, x, biasY);
	biasY = -biasY;

	
	int ratio = 5;
	warp3D.copyTo(warp3DBias);
	warp3DBias.row(0) += biasX*warp3DBias.row(2);
	warp3DBias.row(1) += biasY*warp3DBias.row(2);
	warp3DBias.row(0) /= ratio;
	warp3DBias.row(1) /= ratio;

	warpedImage = Mat::zeros((maxDistDetect - biasY) / ratio, (2 * biasX) / ratio, CV_8UC3);
	warpPerspective(inputImage, warpedImage, warp3DBias, Size((2 * biasX) / ratio, (maxDistDetect - biasY) / ratio)); //将图像进行

	//imshow("warpedImage", warpedImage);
	//waitKey(0);

	boxLen /= ratio;
	boxWidth /= ratio;

	double areaRatio = 2;
	double areaMinThres = boxLen*boxWidth / areaRatio; //面积阈值
	double areaMaxThres = areaMinThres * areaRatio*areaRatio;

	Mat warpedHSV,outputMask;
	cvtColor(warpedImage, warpedHSV, CV_RGB2HSV_FULL);
	//inRange(warpedHSV, Scalar(HL, SL, VL),Scalar(HH, SH, VH), outputMask);
	inRange(warpedHSV, Scalar(interestColor.threshold_value1, interestColor.threshold_value3, interestColor.threshold_value5),
		Scalar(interestColor.threshold_value2, interestColor.threshold_value4, interestColor.threshold_value6), outputMask);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	morphologyEx(outputMask, outputMask, MORPH_OPEN, element);
	/*imshow("detectRegion", outputMask);
	waitKey(10);*/
	int *labels = new int[outputMask.rows*outputMask.cols];
	int maxLabel=bwlabel(&IplImage(outputMask), 8, labels);
	vector<PointCluter> pointCluter(maxLabel);

	//MatIterator_<uchar> itMask = outputMask.begin<uchar>();
	int allNum = outputMask.rows*outputMask.cols;

	for (int k = 0; k<allNum; ++k){
		if (labels[k]==0)
			continue;
		int curLabel = labels[k]-1;
		pointCluter[curLabel].allPoint.push_back(Point2f(k%outputMask.cols, k / outputMask.cols));
		pointCluter[curLabel].meanX += k%outputMask.cols;
		pointCluter[curLabel].meanY += k / outputMask.cols;
		++pointCluter[curLabel].pointNum;
	}
	delete[] labels;

	//删除区域过小的candidate0
	int i = 0;
	while (i<pointCluter.size()) //得到每个cluter的质心
	{
		//cout << pointCluter[i].pointNum << endl;
		if (pointCluter[i].pointNum < areaMinThres || pointCluter[i].pointNum>areaMaxThres){
			pointCluter[i] = pointCluter.back();
			pointCluter.pop_back();
			continue;
		}
		pointCluter[i].boxRect = minAreaRect(pointCluter[i].allPoint); //求取每个区域的最小包围矩形
	//	cout << "矩形角度：" << pointCluter[i].boxRect.angle << endl;
		double LE = pointCluter[i].boxRect.size.height;
		double SE = pointCluter[i].boxRect.size.width;

	/*	double errAngle = pointCluter[i].boxRect.angle - angle+90;
		
		if (abs(errAngle) > 10){
			pointCluter[i] = pointCluter.back();
			pointCluter.pop_back();
			continue;
		}*/

		if (SE > LE)
		{
			double temp = SE;
			SE = LE;
			LE = temp;
		}
		if (SE<=0||abs(LE / SE - boxLen / boxWidth) > 0.8){
			pointCluter[i] = pointCluter.back();
			pointCluter.pop_back();
			continue;
		}

		pointCluter[i].normlize();
		pointCluter[i].refinedX = pointCluter[i].meanX + pointCluter[i].boxRect.center.x;
		pointCluter[i].refinedY = pointCluter[i].meanY + pointCluter[i].boxRect.center.y;

		pointCluter[i].refinedX *= 0.5*ratio;
		pointCluter[i].refinedY *= 0.5*ratio;
		pointCluter[i].refinedX -= biasX;
		pointCluter[i].refinedY -= biasY;
		if (pointCluter[i].refinedY > maxDistDetect || pointCluter[i].refinedY<minDistDetect) //超过三米的点或者小于15cm不进行检测
		{
			pointCluter[i] = pointCluter.back();
			pointCluter.pop_back();
			continue;
		}

		{ //距离约束
			double minDistToLine = 1000000;

			for (int k = 0; k < usefulCluters.size(); ++k){
				double tempA = usefulCluters[k].A_3d, tempB = usefulCluters[k].B_3d, tempC = usefulCluters[k].C_3d;
				double tempDis = abs(tempA*pointCluter[i].refinedX + tempB*pointCluter[i].refinedY + tempC) / sqrt(tempA*tempA + tempB*tempB);
				if (tempDis < minDistToLine)
					minDistToLine = tempDis;
			}

			if (minDistToLine>100){ //50mm
				pointCluter[i] = pointCluter.back();
				pointCluter.pop_back();
				continue;
			}
		}


		double u, v;
		getPointInImageFrom3D(pointCluter[i].refinedX, pointCluter[i].refinedY, u, v);

		if (pointCluter[i].refinedY < labelYPos)
		{
			labelYPos = pointCluter[i].refinedY;
			labelXPos = pointCluter[i].refinedX;
		}

		circle(outPutImg, Point2d(u, v + ROIHEIGHT - 480), 2, Scalar(255, 255, 0), 2);
		++i;
	}

	if (pointCluter.size() > 0)
		return true;
	else
		return false;
}