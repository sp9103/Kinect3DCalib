Kinect3DCalib
=============

Kinect for windows v2 calibration source.

using 3d body information.

*opencv library
*Kienct sdk 2.0 (not 1.8 - kinect v1)

-3D calibration

목적:
 * RGB Calibration에 튜닝이 필요함
 * Depth에선 feature를 뽑기 어려워서 바로 적용이 어려움

대안:
 * RGB calibration하고 Depth로 다시
 * RGB를 Depth 전체 매핑하고 체크보드를 이용한 Calibration
 * 3D calibration 바로 적용
	- point cloud 
		* correspondance를 찾기 어려움
	- skeleton point (V)
		* skeleton noise
		* skeleton pose => 중심축 위주로 계산

방법 : 
 - X1 : 1번 키넥트에서의 좌표 ( 4*1 Vector)
 - X2 : 2번 키넥트에서의 좌표 ( [x,y,z,1]^T )
 - M : transformation matrix ( 4*4 Matrix )
    1. M * X1 = X2
       X1 = inv(M) * X2     (X1 - 4*4Mat )

    2. M * X1 = X2
       M = inv(tran(X1)*X1)*tran(X1)*X2
		- least square ( overfitting 문제 ) 

parameter :
 - N : RANSAC 수행횟수
 - Threshold : Inlier & outlier 구분. (유클리디안 디스턴스)
 - m : 한 루프에 least square를 수행할 샘플 갯수 