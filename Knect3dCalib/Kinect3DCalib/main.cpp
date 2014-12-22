#include "define.h"
#include "BodyCalib.h"
#include "BodyDataLoader.h"

int bodyIdx[USINGJOINT] = {JointType_SpineBase, JointType_SpineMid, JointType_SpineShoulder, JointType_ShoulderLeft, JointType_ShoulderRight,
JointType_HipLeft, JointType_HipRight};

int main(){
	BodyCalib Calib;
	BodyDataLoader DataLoader1, DataLoader2;

	DataLoader1.OpenDataFile("Kinect3.bin", 'r');
	DataLoader2.OpenDataFile("Kinect4.bin", 'r');

	DataLoader1.ReadAllData();
	DataLoader2.ReadAllData();

	for(int i = 0; i < DataLoader1.GetDataCount(); i++){
		BodyJoint temp1, temp2;
		DataLoader1.GetBodyData(&temp1);
		DataLoader2.GetBodyData(&temp2);

		for(int j = 0; j < USINGJOINT; j++){
			Calib.DataStore(temp2.bodyJoint[bodyIdx[j]], temp1.bodyJoint[bodyIdx[j]]);
		}
	}

	int LoopCount = Calib.CalcLoopNUM(0.999, 0.8, 50);
	Calib.InitParam(LoopCount, 0.03, 50);
	Calib.CalcMatrix();

	return 0;
}