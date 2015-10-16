#include <cv.h>
#include <iostream>
#include "elm.h"
#include <set>
using namespace cv;
using namespace std;
//train_data is a mat which has number of cols feature,each feature has the dimision of rows
//ground_truth is a vector contains -1 and +1,which means travelable or not
//weight is the importance of each feature
typedef unsigned int uint;
//extern std::set<int> all_label;
bool elm_ini(unsigned int NumberofInputNeurons,Mat &InputWeight,Mat &Bias){
	uint NumberofHiddenNeurons=50;//NumberofInputNeurons/2;
	InputWeight.create(NumberofHiddenNeurons,NumberofInputNeurons,CV_64F);
	RNG sss;//(unsigned(time(0))) (unsigned(time(0)))
	//randu(InputWeight,Scalar::all(-1),Scalar::all(1));
	sss.fill(InputWeight,RNG::UNIFORM,-0.9999999,1);
	//	randu(InputWeight,Scalar::all(-1),Scalar::all(1));
	Bias.create(NumberofHiddenNeurons, 1, CV_64F);
	sss.fill(Bias,RNG::UNIFORM,0.0000001,1);
	return true;
}

bool elm_train(const Mat& train_data_o,const Mat& ground_truth_o,const Mat & weight_o,
	Mat &InputWeight,Mat &Bias,Mat &OutputWeight){


		Mat train_data=train_data_o.t();
		Mat ground_truth=ground_truth_o.t();//½«¾ØÕó×ªÖÃ
		Mat ground_truth_reverse=-1*ground_truth;
		ground_truth.push_back(ground_truth_reverse);
		
		ground_truth_reverse.release();

		uint NumberofTrainingData=train_data.cols;
		uint NumberofInputNeurons=train_data.rows;
		uint NumberofHiddenNeurons=NumberofInputNeurons/2;
		
		Mat tempH=InputWeight*train_data; //InputWeight is N*M  

		train_data.release(); //  Release input of training data 

		Mat BiasMatrix = Bias*Mat::ones(1, NumberofTrainingData, CV_64FC1);  // Extend the bias matrix BiasofHiddenNeurons to match the demention of H
		
		tempH=tempH+BiasMatrix;
		
		exp(-1*tempH,tempH);  //the exp of  original matrix
		//cout << tempH << endl;
	//	write_mat(tempH);
	//	tempH=tempH+1;        //add one to every element
		tempH=1/(tempH+1);       //ok ,complete the H matrix compution
		
		Mat common_matrix,temp;//=tempH*Mat::diag(weight_o);
		for(int i=0;i<weight_o.rows;++i){
			temp=weight_o.at<double>(i,0)*tempH.col(i);
			temp=temp.t();
			common_matrix.push_back(temp);
			temp.release();
		}
		
		common_matrix=common_matrix.t();
		OutputWeight=common_matrix*tempH.t();//+Mat::eye(NumberofHiddenNeurons,NumberofHiddenNeurons,CV_32F);
		
		for(int i=0;i<OutputWeight.cols;++i){
			OutputWeight.at<double>(i,i)+=0.1;
		}
		
		OutputWeight=OutputWeight.inv();
		
	//	write_mat(OutputWeight);
		
		Mat temp_t=ground_truth.t();
		temp_t.convertTo(temp_t, CV_64F);
		OutputWeight=OutputWeight*(common_matrix*temp_t);//*temp_t *Mat::diag(weight_o)*ground_truth.t();

		return true;
}

bool elm_predict(const Mat &test_data_o,const Mat &InputWeight,const Mat &Bias,const Mat &OutputWeight,Mat &output_label){
	
	Mat test_data=test_data_o.t();
	//write_mat(OutputWeight);
	uint NumberofTrainingData=test_data.cols;
	Mat tempH=InputWeight*test_data; //InputWeight is N*M  
	//Random generate input weights InputWeight (w_i) and biases BiasofHiddenNeurons (b_i) of hidden neurons

	test_data.release(); //  Release input of training data 
	//Mat ind;
	//ind.ones(1,NumberofTrainingData,CV_32F);

	Mat BiasMatrix = Bias*Mat::ones(1, NumberofTrainingData, CV_64F);  // Extend the bias matrix BiasofHiddenNeurons to match the demention of H
	tempH=tempH+BiasMatrix;

	exp(-1*tempH,tempH);  //the exp of  original matrix
	tempH=tempH+1;        //add one to every element
	tempH=1/tempH;       //ok ,complete the H matrix compution
	//;
//	write_mat(tempH.t());
//	write_mat(OutputWeight);
	output_label=tempH.t()* OutputWeight;   //make the prediction
	
	return true;
}