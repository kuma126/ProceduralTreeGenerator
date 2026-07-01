#pragma once

# include<iostream>
# include<string>

// --- 骨格関連 --- //
// //骨格ファイルパス
const std::string SKELETON_CSV_PATH = "../tree/TreeData_Age20_Shoots.csv";
//スケール倍率
const float SKELETON_SCALE = 10.0f;
//障害物エリア内に配置するノード数
const int NODES_SEGMENTS = 1;

// --- PBD関連 --- //
//回数
const int SOLVE = 50;
//ストランド粒子の大きさ
const float STRAND_RADIUS = 0.03f;

//障害物用削除マージン
const float REMOVAL_MAEGIN = 0.01;

//プロファイルの大きさ(基本)(変更時要確認)
const float PROFILE_RADIUS = 0.5f;
//ドロネー三角形の閾値
const float DELAUNAY_THRESHOLD = 0.5f;
//緩いドロネー三角形の閾値(障害物による分岐用)
const float LOOSE_DELAUNAY_THRESHOLD = 20.0f;

// --- メッシュ関連 --- //
const float DETECTION_MARGIN = 0.3f;   // 検出用（粗）
const float SPLIT_MARGIN = 0.2f;  // 分割範囲用（狭）


// 「どれくらいの長さごとにメッシュを切るか」の基準値 (例: 0.1f = 10cmごとに1段)
// この値を大きくすると荒く、小さくすると細かくなる
const float STEP_LENGTH = 0.05f;
// const float OBSTACLE_MARGIN  = 0.02f;



// --- 障害物関連 --- //
//ファイル
const std::string OBJ_PATH = "../obstacles/cylinder_fit_2.obj";
//大きさ?
const glm::vec3 OBJ_SIZE = glm::vec3(1.0f);
//位置
const glm::vec3 OBJ_POS = glm::vec3(-0.1f,0.0f, 0.0f);
