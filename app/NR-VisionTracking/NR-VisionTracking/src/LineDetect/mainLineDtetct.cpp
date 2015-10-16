#include "LineDetect.h"
#include <highgui.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <Windows.h>
#include <direct.h>
using namespace std;
using namespace cv;

const double para[5] = { 0.6000, -0.8000, 0.0010, 0.0005, 327.1323 };

const double cameraPara[] = { 494.3439, 0, 349.0357,
0, 491.5964, 230.9781,
0, 0, 1.0000 };
const double distorPara[] = { 0.1138, -0.0967, -0.0077, 0.0004 };

inline string num2str(int i){
	stringstream ss;
	ss<<i;
	return ss.str();
}

///目录是否存在的检查：
bool  CheckFolderExist()
{
	WIN32_FIND_DATA  wfd;
	bool rValue = false;
	HANDLE hFind = FindFirstFile(L"RoadImage", &wfd);
	if ((hFind != INVALID_HANDLE_VALUE) && (wfd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
	{
		rValue = true;
	}
	FindClose(hFind);
	return rValue;
}

int main(){
	LineDetector lineDetector;
	clock_t st,en,all=0;
	int index=1,num=0;  //1316  974 30 144

	lineDetector.setCamera(cameraPara,distorPara,480,640);
	lineDetector.setExtra(para);

	IplImage* pFrame = NULL;

	//获取摄像头
	CvCapture* pCapture = cvCreateCameraCapture(1);

	if (0){ //CheckFolderExist()
		cout << "RoadImage文件夹已存在，为避免图像文件被覆盖，请重命名该文件夹并重新运行程序。" << endl;
		cvReleaseCapture(&pCapture);
		system("pause");
		return 0;
	}
	else{
		_mkdir("RoadImage");
	}

	namedWindow("outPutImg");
	while(1){
		cout<<"当前处理图像:"<<index<<endl;
		//Mat x2=cv::imread(("F:\\华为\\purpleLabel2\\I ("+num2str(index)+").png").c_str(),1); //F:\\华为\\image\\I ("+num2str(index)+").png
		//Mat x2 = cv::imread(("RoadImage\\I (" + num2str(index) + ").png").c_str(), 1); //F:\\华为\\image\\I ("+num2str(index)+").png
		pFrame = cvQueryFrame(pCapture);
		Mat x2(pFrame);
		imshow("origin", x2);
		waitKey(30);
		cvSaveImage(("RoadImage\\"+num2str(index) + ".png").c_str(), pFrame);
		st=clock();
		Result curResult;
		lineDetector.processImage(x2, true, curResult);
		en=clock();
		all+=en-st;
		
		cout << curResult.angle << " " << curResult.leftDistance << " " << curResult.rightDistance
			<< " " << curResult.labelXpos << " " << curResult.labelYpos << endl;
		
		imshow("outPutImg", lineDetector.outPutImg);
		waitKey(10);
		//++num;
		//cout<<all<<" "<<num<<" "<<en-st<<endl;
		//cout<<"旋转角和不确定度："<<lineDetector.angle<<" "<<lineDetector.cov<<endl<<endl<<endl;
#ifdef _DEBUG
		//
		//imshow("Cur",lineDetector.drawCur);
		//waitKey(30);
#endif // _DEBUG
		index+=1;
	}
	cvReleaseCapture(&pCapture);
}