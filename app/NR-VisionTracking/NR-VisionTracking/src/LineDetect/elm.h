#ifndef elm_h
#define elm_h
#include <cv.h>
bool elm_ini(unsigned int NumberofInputNeurons,cv::Mat &InputWeight,cv::Mat &Bias);
bool elm_train(const cv::Mat& train_data,const cv::Mat& ground_truth,const cv::Mat & weight,
	cv::Mat &InputWeight,cv::Mat &Bias,cv::Mat &OutputWeight);
bool elm_predict(const cv::Mat &test_data_o,const cv::Mat &InputWeight,const cv::Mat &Bias,const cv::Mat &OutputWeight,cv::Mat &output_label);
#endif
