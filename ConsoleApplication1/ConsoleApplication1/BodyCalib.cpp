#include "BodyCalib.h"

BodyCalib::BodyCalib(void)
{
	srand(time(NULL));

	RTMat.eye(4,4, CV_32FC1);
	RMat.eye(3,3, CV_32FC1);
	TMat.zeros(3,1, CV_32FC1);
}

BodyCalib::~BodyCalib(void)
{
	RMat.release();
	TMat.release();
	RTMat.release();
}

void BodyCalib::DataStore(cv::Point3f first, cv::Point3f second){
	std::pair<cv::Point3f, cv::Point3f> tDataPair;
	tDataPair.first = first;
	tDataPair.second = second;

	DataSet.push_back(tDataPair);
}

void BodyCalib::SelectRandomNum(int *dst, int sampleCount){
	int selectedCount = 0;

	for(int i = 0; i < 4; i ++){
		int rNum = rand() % sampleCount;

		for(int j = 0; j < selectedCount; j++){
			if(dst[j] == rNum){
				i--;
				selectedCount--;
				break;
			}
		}

		dst[selectedCount] = rNum;
		selectedCount++;
	}
}

void BodyCalib::printMat(cv::Mat src){
	for(int i = 0; i < src.rows; i++){
		for(int j = 0; j < src.cols; j++){
			printf("%f ", src.at<float>(i,j));
		}
		printf("\n");
	}
}

void BodyCalib::CalcMatrix(int num){
	//RANSAC
}