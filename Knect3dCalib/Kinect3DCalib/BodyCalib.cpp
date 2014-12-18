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

void BodyCalib::CalcMatrix(int num, double threshold){
	int tLoopCount = 0;
	int NumInlier = -1;
	float averError;

	int DataIdx[4];
	cv::Mat X1, X2;
	cv::Mat tempM;
	
	X1.zeros(4,4, CV_32FC1);
	X2.zeros(4,4, CV_32FC1);

	//RANSAC
	while(1){
		if(tLoopCount > num)		break;

		SelectRandomNum(DataIdx, DataSet.size());

		CreateMat(DataIdx, &X1, &X2);
		
		//Determinat zero => can not calculate inverse function.
		double tDet = cv::determinant(X1);
		if(abs(tDet) < 0.00001)
			continue;

		//M * X1 = X2. Calculate matrix M
		tempM = X1.inv() * X2;

		int InlierCount = CalcInlierCount(tempM, threshold, &averError);

		if(NumInlier < InlierCount){
			NumInlier = InlierCount;
			RTMat = tempM;
		}

		tLoopCount++;
	}

	X1.release();
	X2.release();

	printf("RANSAC Complete!\n");
	DecomposeRTMat();

	//ERROR
	printf("Average Error : %f cm\n", averError*100.f);
}

void BodyCalib::CreateMat( int *idxarr, cv::Mat *Mat1, cv::Mat *Mat2 )
{
	cv::Point3f firstPoint, secondPoint;

	//Matrix create
	for(int i = 0; i < 4; i++){
		firstPoint = DataSet.at(idxarr[i]).first;
		secondPoint = DataSet.at(idxarr[i]).second;

		Mat1->at<float>(0,i) = firstPoint.x;
		Mat1->at<float>(1,i) = firstPoint.y;
		Mat1->at<float>(2,i) = firstPoint.z;
		Mat1->at<float>(3,i) = 1.0f;

		Mat2->at<float>(0,i) = secondPoint.x;
		Mat2->at<float>(1,i) = secondPoint.y;
		Mat2->at<float>(2,i) = secondPoint.z;
		Mat2->at<float>(3,i) = 1.0f;
	}
}

int BodyCalib::CalcInlierCount(cv::Mat RT, float threshold, float *averError){
	int iCount = 0;
	*averError = 0.0f;

	for(int i = 0; i < DataSet.size(); i++){
		float DataDist = CalcDist(RT, i);
		*averError += DataDist;

		if(DataDist < threshold)
			iCount++;
	}

	*averError /= (float)DataSet.size();

	return iCount;
}

float BodyCalib::CalcDist(cv::Mat RT, int idx){
	// M * X1 = X2
	cv::Point3f x1 = DataSet.at(idx).first;
	cv::Point3f x2 = DataSet.at(idx).second;

	cv::Mat x1vec, x1vec_after;
	x1vec.zeros(3,1, CV_32FC1);
	x1vec_after.zeros(3,1, CV_32FC1);

	x1vec.at<float>(0,0) = x1.x;
	x1vec.at<float>(1,0) = x1.y;
	x1vec.at<float>(2,0) = x1.z;
	x1vec.at<float>(3,0) = 1.0f;

	//x1' = M*x1
	x1vec_after = RT*x1vec;

	//Euclidean Distance
	float tdist = sqrt(pow(x1vec.at<float>(0) - x2.x,2) + pow(x1vec.at<float>(1) - x2.y,2) + pow(x1vec.at<float>(2) - x2.z,2));

	x1vec.release();
	x1vec_after.release();

	return tdist;
}

void BodyCalib::DecomposeRTMat(){
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			RMat.at<float>(i,j) = RTMat.at<float>(i,j);
		}
		TMat.at<float>(i,0) = RTMat.at<float>(i,4);
	}
}