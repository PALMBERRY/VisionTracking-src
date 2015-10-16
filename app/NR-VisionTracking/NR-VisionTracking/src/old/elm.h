#ifndef elm_h
#define elm_h
#include <cv.h>
using namespace cv;
bool elm_ini(unsigned int NumberofInputNeurons,Mat &InputWeight,Mat &Bias);
bool elm_train(const Mat& train_data,const Mat& ground_truth,const Mat & weight,
	Mat &InputWeight,Mat &Bias,Mat &OutputWeight);
bool elm_predict(const Mat &test_data_o,const Mat &InputWeight,const Mat &Bias,const Mat &OutputWeight,Mat &output_label);
#endif
