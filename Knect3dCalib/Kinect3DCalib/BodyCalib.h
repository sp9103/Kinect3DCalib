/************************************************************************/
/*USAGE :
1. DataStore
2. InitParam - LoopCount�� ���ϱ� ���ؼ� CalcLoopNUM�� ����� �� ����
3. CalcMatrix
4. GetMatrix*/
/************************************************************************/
#include "define.h"

class BodyCalib
{
public:
	BodyCalib(void);
	~BodyCalib(void);

	/*M * X1 = X2
	X2 is basis space*/
	void DataStore(cv::Point3f first, cv::Point3f second);

	//RANSAC implement, num : ����Ƚ��
	void CalcMatrix();

	//Initialize
	void InitParam(int LoopCount, float Threshold, int samplecount);

	/*N ���
	p : target probability
	alpha : inlier ratio
	sampleCount : 
	*/
	int CalcLoopNUM(float p, float alpha, int samplecount);

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
	//�װ��� ���� �ε��� ����
	void SelectRandomNum(int *Box);

	void printMat(cv::Mat src);

	// ex. idx == 1 . first row 
	void CreateRefMat(int *idxarr, cv::Mat* Mat1);
	void CreateTargetMat(int *idxarr, cv::Mat* Mat2, int idx);

	// RT�� �� ���� ��ȯ
	float CalcDist(cv::Mat RT, int idx);

	//inlier ���� ��ȯ
	int CalcInlierCount(cv::Mat RT, float threshold, float *averError);

	//��ü ���� ���
	void CalcAverError();

	//Matrix ����
	void DecomposeRTMat();
};