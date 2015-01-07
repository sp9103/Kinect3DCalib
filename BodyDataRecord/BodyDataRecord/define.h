#ifndef DEFINE_HEADER_FILE_
#define DEFINE_HEADER_FILE_

#include <stdio.h>
#include <opencv.hpp>
#include <time.h>

#define SWAP(a,b,t) ((t)=(a), (a)=(b), (b)=(t))

//typedef enum _JointType
//{
//    JointType_SpineBase = 0,
//    JointType_SpineMid = 1,
//    JointType_Neck = 2,
//    JointType_Head = 3,
//    JointType_ShoulderLeft = 4,
//    JointType_ElbowLeft = 5,
//    JointType_WristLeft = 6,
//    JointType_HandLeft = 7,
//    JointType_ShoulderRight = 8,
//    JointType_ElbowRight = 9,
//    JointType_WristRight = 10,
//    JointType_HandRight = 11,
//    JointType_HipLeft = 12,
//    JointType_KneeLeft = 13,
//    JointType_AnkleLeft = 14,
//    JointType_FootLeft = 15,
//    JointType_HipRight = 16,
//    JointType_KneeRight = 17,
//    JointType_AnkleRight = 18,
//    JointType_FootRight = 19,
//    JointType_SpineShoulder = 20,
//    JointType_HandTipLeft = 21,
//    JointType_ThumbLeft = 22,
//    JointType_HandTipRight = 23,
//    JointType_ThumbRight = 24,
//    JointType_Count = 25
//} JointType/*, JointType_SpineBase, JointType_SpineMid, JointType_Neck, JointType_Head, JointType_ShoulderLeft, JointType_ElbowLeft, JointType_WristLeft, JointType_HandLeft, JointType_ShoulderRight, JointType_ElbowRight, JointType_WristRight, JointType_HandRight, JointType_HipLeft, JointType_KneeLeft, JointType_AnkleLeft, JointType_FootLeft, JointType_HipRight, JointType_KneeRight, JointType_AnkleRight, JointType_FootRight, JointType_SpineShoulder, JointType_HandTipLeft, JointType_ThumbLeft, JointType_HandTipRight, JointType_ThumbRight, JointType_Count*/;

typedef struct BodyJoint{
	cv::Point3f bodyJoint[25];
}BodyJoint;

#endif