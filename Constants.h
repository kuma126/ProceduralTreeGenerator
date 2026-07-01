#pragma once

#include <cmath>

//円周率の定義
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//ウィンドウ設定
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 800;

//シミュレーション設定
const int NUM_PARTICLES = 450;
const float MOVEMENT_ALPHA = 0.01f; //移動速度を制御する係数

//PBD関連の定数
const float ATTRACTION_STIFFNESS = 0.1f; //引力の強さ
const float CENTER_ATTRACTION = 0.01f;//重心への引力

//ドロネー三角形分割用の閾値
const float DELAUNAY_THRESHOLD;

