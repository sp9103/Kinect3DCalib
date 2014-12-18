#include "define.h"

class BodyCalib
{
public:
	BodyCalib(void);
	~BodyCalib(void);

	void DataStore(cv::Point3f first, cv::Point3f second);

	//RANSAC implement, num : ½ÃÇàÈ½¼ö
	void CalcMatrix(int num);

private:
	cv::Mat RMat;
	cv::Mat TMat;
	cv::Mat RTMat;

	//Data set
	std::vector<std::pair<cv::Point3f, cv::Point3f>> DataSet;
	//³×°³ÀÇ ·£´ý ÀÎµ¦½º ¼±ÅÃ
	void SelectRandomNum(int *dst, int sampleCount);

	void printMat(cv::Mat src);
};