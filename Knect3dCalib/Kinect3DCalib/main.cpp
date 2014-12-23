#include "define.h"
#include "BodyCalib.h"
#include "BodyDataLoader.h"

#define SAMPLECOUNT 20

int bodyIdx[USINGJOINT] = {JointType_SpineBase, JointType_SpineMid, JointType_SpineShoulder, JointType_ShoulderLeft, JointType_ShoulderRight,
JointType_HipLeft, JointType_HipRight};

int main(){
	BodyCalib Calib;
	BodyDataLoader DataLoader1, DataLoader2;
	int iCount = 0, jCount = 0;

	DataLoader1.OpenDataFile("Kienct3BodyData.bin", 'r');
	DataLoader2.OpenDataFile("Kienct4BodyData.bin", 'r');

	DataLoader1.ReadAllData();
	DataLoader2.ReadAllData();

	int tDataCount = DataLoader1.GetDataCount();

	for(int i = 0; i < tDataCount; i++){
		BodyJoint temp1, temp2;
		DataLoader1.GetBodyData(&temp1);
		DataLoader2.GetBodyData(&temp2);

		for(int j = 0; j < USINGJOINT; j++){
			Calib.DataStore(temp2.bodyJoint[bodyIdx[j]], temp1.bodyJoint[bodyIdx[j]]);
			jCount++;
		}
		iCount++;
	}

	int LoopCount = Calib.CalcLoopNUM(0.999, 0.8, SAMPLECOUNT);
	printf("Loop Count : %d\n", LoopCount);
	Calib.InitParam(LoopCount, 0.02, SAMPLECOUNT);
	Calib.CalcMatrix();

	return 0;
}