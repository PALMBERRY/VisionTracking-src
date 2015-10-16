#pragma once
#include <vector>
#include <cv.h>
#include "lsd_opencv.hpp"

struct cluter{
	cv::Mat center;
	cv::Mat cov;
	cv::Mat covinv;
	float sampleNum;
	//bool protect_flag; //in history ,true means it has been updated
	//in curr,true means it has helped history to be updated
	cluter() :sampleNum(0), center(cv::Mat::zeros(1, 3, CV_64FC1)), cov(cv::Mat::zeros(3, 3, CV_64FC1)){}
};

struct LineCluter{
	double lenAll;
	double meanA;
	double meanB;
	double meanC;
	double angle; //只有最后一步计算角度会用到
	double score; //高斯得分

	double A_3d;
	double B_3d;
	double C_3d;
	cv::Point2d bottomPixelLoc;
	cv::Point2d upPixelLoc;

	std::vector<int> indexOfLine; //用一般式表示直线

	bool operator <(const LineCluter &other) const
	{
		return lenAll>other.lenAll;
	}
};

struct Line
{
	cv::Vec4i line;
	double A;
	double B;
	double C;
	double len;
	//int cluterID;
};

struct Result{
	double angle;
	double leftDistance;
	double rightDistance;
	double labelAngle;
	double labelDistance;
	Result() :angle(-1), leftDistance(-1), rightDistance(-1), labelAngle(-1), labelDistance(-1){};
	void reset(){
		angle = -1; leftDistance = -1; rightDistance = -1; labelAngle = -1; labelDistance = -1;
	}
};

class LineDetector{
private:
	cv::Mat InputWeight, Bias, OutputWeight;

	std::vector<LineCluter> usefulCluters;
	std::vector<Line> lineGetInf; //计算得到相关信息后的线段
	std::vector<LineCluter> lineCluters;
	std::vector<cv::Vec4i> lines; //LSD算法计算得到的直线

	cv::Ptr<cv::LineSegmentDetector> lsd;
	cv::Rect roi;		//截取的兴趣区域
	cv::Mat roiImageC;
	int imageHeight;
	int imageWidth;

	int trainSampleIndex;
	cv::Mat cameraMatrix;

	cv::Mat distMatrix;
	//Mat KRTI;
	cv::Mat warp3D;
	cv::Mat warp3DInv;
	cv::Mat warp3DBias;
	double lengthAll;
	inline void drawLine();
	void findPt(const double &A, const double &B, const double &C,std::vector<cv::Point2d> &pt);
	void sampleLineToFit(int index1, int index2, double &A, double &B, double &C);
	bool loadModel();
	void getGoodLinesPair(std::vector<LineCluter> lineCluters);
	void calAngleDist();
	void gaussTrainSampleExtract(int picIndex);
	void showLine(const std::vector<cv::Vec4i> &interestLine);
	void showLineCluter(const std::vector<LineCluter> &usefulCluters, const std::string windowName);
	double angleLeftBound;
	double angleRightBound;

	int leftNearestIndex, rightNearestIndex;
	double angle;
	double leftNearestDis, rightNearestDis;

	double para[5];// = { 0.680384595505694, 0.731024173771724, -0.0403536713356341, -0.0324351796755852, 330.000002721088 };
	double cameraPara[9];// = { 713.71495, 0, 330.40424, 0, 713.35601, 260.00015, 0, 0, 1 };
	double distorPara[4]; //= { -0.21476, 0.10375, -0.00125, 0.00119 };

	void updatePara();
	void clearAll(); //新图像进入后，将所有关于前一帧图像的信息删除
	void lineClutersIni();
	void fuseCluter();
	void findBestLine();
	void trainELM();
	void getPoint3D(double u, double v, double &x, double &y){
		double axisInImage[3] = { u, v, 1 };
		cv::Mat axisImage(3, 1, CV_64FC1, axisInImage);
		cv::Mat_<double> divder = warp3D.row(2)*axisImage; double divderD = divder(0, 0);
		cv::Mat_<double> x_ = warp3D.row(0)*axisImage / divderD;
		cv::Mat_<double> y_ = warp3D.row(1)*axisImage / divderD;
		x = x_(0, 0);
		y = y_(0, 0);
	};

	void getPointInImageFrom3D(double x, double y,double &u, double &v){   //可用于得到兴趣区域的范围
		double axisInWorld[3] = { x, y, 1 };
		cv::Mat axisWorld(3, 1, CV_64FC1, axisInWorld);
		cv::Mat_<double> divder = warp3DInv.row(2)*axisWorld; double divderD = divder(0, 0);
		cv::Mat_<double> u_ = warp3DInv.row(0)*axisWorld / divderD;
		cv::Mat_<double> v_ = warp3DInv.row(1)*axisWorld / divderD;
		u = u_(0, 0);
		v = v_(0, 0);
	}

public:
	LineDetector();
	void setCamera(const double *K, const double *distor, const int _imageHeight, const int _imageWidth);
	void setExtra(const double *RT);
	cv::Mat outPutImg;
	void processImage(const cv::Mat &image, bool labelDetectFlag,Result &detectResult,bool trainMode=false, int imageIndex=-1);
};