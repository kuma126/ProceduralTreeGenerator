#pragma once 

#include <vector>

//2次元の点を表す構造体
struct Point3D {
	float x, y, z;
	float r, g, b;
	float prev_x, prev_y;
	bool  initially_inside;

	//各パーティクルが記憶する目標点の座標
	float target_x;
	float target_y;

	bool isOnObstacleBoundary = false;//障害物境界上にいるかを示すフラグ

	//ライティング用
	float nx, ny, nz;

	int id;
};