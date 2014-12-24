#include "define.h"
#include "BodyCalib.h"
#include "BodyDataLoader.h"

#define SAMPLECOUNT 40

int bodyIdx[USINGJOINT] = {JointType_SpineBase, JointType_SpineMid, JointType_SpineShoulder, JointType_ShoulderLeft, JointType_ShoulderRight,
	JointType_HipLeft, JointType_HipRight, JointType_KneeRight, JointType_KneeLeft, JointType_ElbowLeft, JointType_ElbowRight};

int main(){
	BodyCalib Calib;
	BodyDataLoader DataLoader1, DataLoader2;
	int iCount = 0, jCount = 0;

	DataLoader1.OpenDataFile("3-5/Kienct3BodyData.bin", 'r');
	DataLoader2.OpenDataFile("3-5/Kienct5BodyData.bin", 'r');

	DataLoader1.ReadAllData();
	DataLoader2.ReadAllData();

	int tDataCount = DataLoader1.GetDataCount();

	for(int i = 0; i < tDataCount; i++){
		BodyJoint temp1, temp2;
		DataLoader1.GetBodyData(&temp1);
		DataLoader2.GetBodyData(&temp2);

		if(i == 7)
			continue;

		for(int j = 0; j < USINGJOINT; j++){
			Calib.DataStore(temp2.bodyJoint[bodyIdx[j]], temp1.bodyJoint[bodyIdx[j]]);
			printf("[%f %f %f]\n", temp1.bodyJoint[bodyIdx[j]].x, temp1.bodyJoint[bodyIdx[j]].y, temp1.bodyJoint[bodyIdx[j]].z);
			jCount++;
		}
		printf("\n");
		iCount++;
	}

	printf("Inserted Data Count : %d\n", jCount);

	int LoopCount = Calib.CalcLoopNUM(0.999, 0.8, SAMPLECOUNT);
	printf("Loop Count : %d\n", LoopCount);
	Calib.InitParam(LoopCount, 0.03, SAMPLECOUNT);
	Calib.CalcMatrix();

	return 0;
}