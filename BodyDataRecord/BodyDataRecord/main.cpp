#include "stdafx.h"

#include "KinectConnecter.h"
#include "BodyDataLoader.h"

using namespace cv;

void SkelToBody(SkeletonInfo *src, BodyJoint *dst){

	for(int i = 0; i < JointType_Count; i++){
		dst->bodyJoint[i].x = src[0].InfoBody[0].JointPos[i].Position.X;
		dst->bodyJoint[i].y = src[0].InfoBody[0].JointPos[i].Position.Y;
		dst->bodyJoint[i].z = src[0].InfoBody[0].JointPos[i].Position.Z;
	}
}

int main(){
	Mat KinectColorImage;			//Kinect Color Image
	Mat KinectDepthImage;
	Mat KinectBodyIdxImage;
	int SavedDataCount = 0;

	KinectColorImage.create(KINECT_COLOR_HEIGHT, KINECT_COLOR_WIDTH, CV_8UC4);			//Kinect Color Image format BGRA 4 channel image
	KinectDepthImage.create(KINECT_DEPTH_HEIGHT, KINECT_DEPTH_WIDTH, CV_8UC4);			//Kinect Depth Image format BGRA 4 Channel image
	//KinectBodyIdxImage.create(KINECT_DEPTH_HEIGHT, KINECT_DEPTH_WIDTH, CV_8UC4);		//Kinect Depth Image format Gray 1 Channel image
	SkeletonInfo m_SkeletonInfo[NUM_KINECTS];

	KinectConnecter Kinect;
	BodyDataLoader DataLoader;

	int tid;
	printf("Insert Kinect ID : ");
	scanf("%d", &tid);
	
	char buf[256];
	sprintf(buf, "Kienct%dBodyData.bin", tid);

	Kinect.KinectInitialize();
	DataLoader.OpenDataFile(buf, 'w');

	namedWindow("KinectColorFrame", CV_WINDOW_KEEPRATIO);
	namedWindow("KinectDepthFrame", CV_WINDOW_KEEPRATIO);

	while(1){
		Kinect.GetColorImage(&KinectColorImage);
		Kinect.GetDepthImage(&KinectDepthImage);
		Kinect.GetSkeletonPos(m_SkeletonInfo, &KinectDepthImage, 1);

		imshow("KinectColorFrame", KinectColorImage);
		imshow("KinectDepthFrame", KinectDepthImage);

		char keyinput = waitKey(33);

		if(keyinput == 's'){
			//Save Data
			SavedDataCount++;
			printf("%d Data Saved!\n", SavedDataCount);


			BodyJoint temp;
			SkelToBody(m_SkeletonInfo, &temp);
			DataLoader.WriteData(&temp);
		}
		else if(keyinput == 27){
			break;
		}
	}

	Kinect.KinectDestroy();
	DataLoader.CloseDataFile();

	destroyAllWindows();

	return 0;
}