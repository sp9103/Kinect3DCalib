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

void BodyCalib::InitParam(int LoopCount, float Threshold, int samplecount){
	m_N = LoopCount;
	m_Threshold = Threshold;
	m_m = samplecount;
}

void BodyCalib::DataStore(cv::Point3f first, cv::Point3f second){
	std::pair<cv::Point3f, cv::Point3f> tDataPair;
	tDataPair.first = first;
	tDataPair.second = second;

	DataSet.push_back(tDataPair);
}

void BodyCalib::SelectRandomNum(int *randBox){
	for(int i = 0; i < DataSet.size(); i++){
		int tidx = rand()%DataSet.size();
		int temp;

		SWAP(randBox[i], randBox[tidx], temp);
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

void BodyCalib::CalcMatrix(){
	int tLoopCount = 0;
	int NumInlier = -1;
	float averError, bestError = 999.f;

	cv::Mat X1, X2, X1TX1;
	cv::Mat tempM, tM1;

	//X1.zeros(m_m, 4, CV_32FC1);
	//X2.zeros(m_m, 1, CV_32FC1);
	//X1TX1.zeros(4, 4, CV_32FC1);
	//tempM.zeros(4, 4, CV_32FC1);
	//tM1.zeros(1,4, CV_32FC1);

	X1.create(m_m,4,CV_32FC1);
	X2.create(m_m,1,CV_32FC1);
	X1TX1.create(4,4,CV_32FC1);
	tempM.create(4,4,CV_32FC1);
	tM1.create(1,4,CV_32FC1);

	//RandBOx make
	int *randBox = (int*)malloc(sizeof(int)*DataSet.size());
	for(int i = 0; i < DataSet.size(); i++){
		randBox[i] = i;
	}

	//RANSAC
	while(1){
		if(tLoopCount > m_N)		break;

		SelectRandomNum(randBox);

		//Least Square solve - °¢ row º°·Î
		for(int i = 0; i < 3; i++){
			CreateMat(randBox, &X1, &X2, i);
			X1TX1 = X1.t()*X1;
			cv::transpose(X1TX1.inv(cv::DECOMP_SVD) * X1.t() * X2, tM1);

			for(int j = 0; j < 4; j++){
				tempM.at<float>(i,j) = tM1.at<float>(0,j);
			}
		}

		tempM.at<float>(3,0) = 0.0f;
		tempM.at<float>(3,1) = 0.0f;
		tempM.at<float>(3,2) = 0.0f;
		tempM.at<float>(3,3) = 1.0f;

		//CreateMat(randBox, &X1, &X2);
		//
		//X1TX1 = X1.t()*X1;
		////////////////////////////////////////////////////////
		////Determinat zero => can not calculate inverse function.
		//double tDet = cv::determinant(X1TX1);
		//if(abs(tDet) < 0.00001)
		//	continue;

		////M * X1 = X2. Calculate matrix M
		//tempM =  X2 * X1TX1.inv()* X1.t();

		int InlierCount = CalcInlierCount(tempM, m_Threshold, &averError);

		if(NumInlier <= InlierCount){
			if(averError*100.f < bestError){
				NumInlier = InlierCount;
				tempM.copyTo(RTMat);
				bestError = averError*100.f;
			}
		}

		tLoopCount++;
		printf("\n[%d] Loop complete!\n", tLoopCount);
		printf("- Inlier count : %d\n", InlierCount);
		printf("- Average Err : %fcm\n\n", averError*100.f);
		printMat(tempM);
	}

	X1.release();
	X2.release();
	tempM.release();
	tM1.release();

	printf("\nRANSAC Complete!\n");
	printf("Result Matri : \n");
	printMat(RTMat);

	//DecomposeRTMat();

	//ERROR
	printf("Average Error : %f cm\n", bestError);
	printf("Inlier Count : %d\n", NumInlier);

	free(randBox);
}

void BodyCalib::CreateMat( int *idxarr, cv::Mat *Mat1, cv::Mat *Mat2, int idx)
{
	cv::Point3f firstPoint, secondPoint;

	//Matrix create
	for(int i = 0; i < m_m; i++){
		firstPoint = DataSet.at(idxarr[i]).first;
		secondPoint = DataSet.at(idxarr[i]).second;

		Mat1->at<float>(i,0) = firstPoint.x;
		Mat1->at<float>(i,1) = firstPoint.y;
		Mat1->at<float>(i,2) = firstPoint.z;
		Mat1->at<float>(i,3) = 1.0f;

		switch(idx){
		case 0:
			Mat2->at<float>(i,0) = secondPoint.x;
			break;
		case 1:
			Mat2->at<float>(i,0) = secondPoint.y;
			break;
		case 2:
			Mat2->at<float>(i,0) = secondPoint.z;
			break;
		}
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
	x1vec.create(4,1, CV_32FC1);
	x1vec_after.create(4,1, CV_32FC1);

	x1vec.at<float>(0,0) = x1.x;
	x1vec.at<float>(1,0) = x1.y;
	x1vec.at<float>(2,0) = x1.z;
	x1vec.at<float>(3,0) = 1.0f;

	//x1' = M*x1
	x1vec_after = RT*x1vec;

	cv::Point3f x1after;
	x1after.x = x1vec_after.at<float>(0,0);
	x1after.y = x1vec_after.at<float>(1,0);
	x1after.z = x1vec_after.at<float>(2,0);

	//Euclidean Distance
	float tdist = sqrt(pow(x1after.x - x2.x,2) + pow(x1after.y - x2.y,2) + pow(x1after.z - x2.z,2));

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

cv::Mat BodyCalib::GetRMatrix(){
	return RMat.clone();
}

cv::Mat BodyCalib::GetTMatrix(){
	return TMat.clone();
}

cv::Mat BodyCalib::GetRTMatrix(){
	return RTMat.clone();
}

int BodyCalib::CalcLoopNUM(float p, float alpha, int samplecount){
	return log(1.0f - p) / log(1.0f - pow(alpha, samplecount));
}