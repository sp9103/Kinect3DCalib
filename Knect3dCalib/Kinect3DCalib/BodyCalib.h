#include "define.h"

class BodyCalib
{
public:
	BodyCalib(void);
	~BodyCalib(void);

	/*M * X1 = X2
	X2 is basis space*/
	void DataStore(cv::Point3f first, cv::Point3f second);

	//RANSAC implement, num : 시행횟수
	void CalcMatrix();

	//Initialize
	void InitParam(int LoopCount, float Threshold, int samplecount);

	//Get Matrix
	cv::Mat GetRMatrix();
	cv::Mat GetTMatrix();
	cv::Mat GetRTMatrix();

	//

private:
	cv::Mat RMat;
	cv::Mat TMat;
	cv::Mat RTMat;

	//Parameter
	int m_N;							//number of loop count
	float m_Threshold;					//inlier, outlier bound
	int m_m;							//pick sample count

	//Data set
	std::vector<std::pair<cv::Point3f, cv::Point3f>> DataSet;
	//네개의 랜덤 인덱스 선택
	void SelectRandomNum(int *Box);

	void printMat(cv::Mat src);

	void CreateMat(int *idxarr, cv::Mat *Mat1, cv::Mat *Mat2);

	// RT한 뒤 오차 반환
	float CalcDist(cv::Mat RT, int idx);

	//inlier 갯수 반환
	int CalcInlierCount(cv::Mat RT, float threshold, float *averError);

	//전체 에러 계산
	void CalcAverError();

	//Matrix 분해
	void DecomposeRTMat();
};