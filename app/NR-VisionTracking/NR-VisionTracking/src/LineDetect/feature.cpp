#include <cv.h>

#define PI 3.1415926535897932
#define POW(nBit)   (1 << (nBit))

int map_x[]={1,1,0,-1,-1,-1,0,1};
int map_y[]={0,-1,-1,-1,0,1,1,1};
int  UniformPattern10[256]={    
	1,   2,   3,   4,   5,   0,   6,   7,   8,   0,   0,   0,   3,   0,   4,   5,
	2,   0,   0,   0,   0,   0,   0,   0,   3,   0,   0,   0,   4,   0,   5,   6,
	2,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	3,   0,   0,   0,   0,   0,   0,   0,   4,   0,   0,   0,   5,   0,   6,   7,
	2,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	3,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	4,   0,   0,   0,   0,   0,   0,   0,   5,   0,   0,   0,   6,   0,   7,   8,
	2,   3,   0,   4,   0,   0,   0,   5,   0,   0,   0,   0,   0,   0,   0,   6,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   7,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   8,//191
	3,   4,   0,   5,   0,   0,   0,   6,   0,   0,   0,   0,   0,   0,   0,   7,//207
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   8,//223
	4,   5,   0,   6,   0,   0,   0,   7,   0,   0,   0,   0,   0,   0,   0,   8,//239
	5,   6,   0,   7,   0,   0,   0,   8,   6,   7,   0,   8,   7,   8,   8,   9
};

int  UniformPattern59[256]={    
	1,   2,   3,   4,   5,   0,   6,   7,   8,   0,   0,   0,   9,   0,  10,  11,
	12,   0,   0,   0,   0,   0,   0,   0,  13,   0,   0,   0,  14,   0,  15,  16,
	17,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	18,   0,   0,   0,   0,   0,   0,   0,  19,   0,   0,   0,  20,   0,  21,  22,
	23,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	24,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	25,   0,   0,   0,   0,   0,   0,   0,  26,   0,   0,   0,  27,   0,  28,  29,
	30,  31,   0,  32,   0,   0,   0,  33,   0,   0,   0,   0,   0,   0,   0,  34,
	0,    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  35,
	0,    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  36,
	37,  38,   0,  39,   0,   0,   0,  40,   0,   0,   0,   0,   0,   0,   0,  41,
	0,    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  42,
	43,  44,   0,  45,   0,   0,   0,  46,   0,   0,   0,   0,   0,   0,   0,  47,
	48,  49,   0,  50,   0,   0,   0,  51,  52,  53,   0,  54,  55,  56,  57,  58
};

void build_lbp_map(const cv::Mat &image,cv::Mat &LBP_map){
	using namespace cv;
	cv::Mat gray_image;
	cvtColor(image,gray_image,CV_RGB2GRAY); //convert image to gray
	LBP_map=gray_image;//to save LBP_map
	int i,j,p,center_byte,current_byte,X,Y;
	int BasicLBP = 0;	int FeaBin = 0;
	for(i=0;i<gray_image.rows;++i)
		for(j=0;j<gray_image.cols;++j){
			if(i==0||i==gray_image.rows-1||j==0||j==gray_image.cols-1){
			//	LBP_map.at<uchar>(i,j)=255;
				continue;
			}
			center_byte=(int)gray_image.at<uchar>(i,j);
			BasicLBP = 0;	FeaBin = 0;
			for(p = 0; p < 8; ++p)
			{
			//	X = int (j + cos( PI * p/4) + 0.5);
			//	Y = int (i - sin( PI * p/4) + 0.5);
				X=i+map_x[p];
				Y=j+map_y[p];
				current_byte =(int)gray_image.at<uchar>(X,Y);
				if (abs(current_byte - center_byte)>3)  //abs(current_byte-center_byte)>3
					BasicLBP += POW ( FeaBin);   //这一位的BasicLBP值统计完成
				FeaBin++;
			}
			LBP_map.at<uchar>(i,j)=uchar(UniformPattern10[BasicLBP]);
		}
	LBP_map.row(1).copyTo(LBP_map.row(0));
	LBP_map.row(LBP_map.rows-2).copyTo(LBP_map.row(LBP_map.rows-1));
	LBP_map.col(1).copyTo(LBP_map.col(0));
	LBP_map.col(LBP_map.cols-2).copyTo(LBP_map.col(LBP_map.cols-1));
}
