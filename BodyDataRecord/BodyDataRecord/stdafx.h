#pragma once

#include <Windows.h>
#include <stdio.h>
#include <opencv.hpp>
#include <Kinect.h>

#define KINECT_COLOR_WIDTH		1920
#define KINECT_COLOR_HEIGHT		1080
#define KINECT_DEPTH_WIDTH		512
#define KINECT_DEPTH_HEIGHT		424
#define NUM_KINECTS				1

#define OPENCV_WAIT_DELAY		10

#define BODY_COUNT				6

#define PI						3.141592653589

////Single Body Structure;
//typedef struct BodyInfo{
//	Joint JointPos[JointType_Count];
//	UINT64 BodyID;
//}BodyInfo;

////Store sensor out Body information
//typedef struct SkeletonInfo{
//	int Count;												//현재 추적하고 있는 스켈레톤 갯수
//	BodyInfo InfoBody[BODY_COUNT];
//}SkeletonInfo;

//Single Body Structure;
typedef struct BodyInfo{
	Joint JointPos[JointType_Count];
	cv::Point2d jointPoints[JointType_Count];
	UINT64 BodyID;
}BodyInfo;

//Store sensor out Body information
typedef struct SkeletonInfo{
	int Kinect_ID;
	int Count;												//현재 추적하고 있는 스켈레톤 갯수
	SYSTEMTIME	st;
	BodyInfo InfoBody[BODY_COUNT];
}SkeletonInfo;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}