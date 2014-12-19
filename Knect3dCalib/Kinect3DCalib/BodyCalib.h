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
	void CalcMatrix(int num, double threshold);

	//Get Matrix
	cv::Mat GetRMatrix();
	cv::Mat GetTMatrix();
	cv::Mat GetRTMatrix();

	//

private:
	cv::Mat RMat;
	cv::Mat TMat;
	cv::Mat RTMat;

	//Data set
	std::vector<std::pair<cv::Point3f, cv::Point3f>> DataSet;
	//�װ��� ���� �ε��� ����
	void SelectRandomNum(int *dst, int sampleCount);

	void printMat(cv::Mat src);

	void CreateMat(int *idxarr, cv::Mat *Mat1, cv::Mat *Mat2);

	// RT�� �� ���� ��ȯ
	float CalcDist(cv::Mat RT, int idx);

	//inlier ���� ��ȯ
	int CalcInlierCount(cv::Mat RT, float threshold, float *averError);

	//��ü ���� ���
	void CalcAverError();

	//Matrix ����
	void DecomposeRTMat();
};