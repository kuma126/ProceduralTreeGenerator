#include "TreeSystem.h"
#include "ParticleSystem.h"
#include "GeometryUtils.h"
#include "OBJExporter.h"
#include "MeshObstacle.h"
#include "CurveGenerator.h"
#include "Parameter.h"
#include <iostream>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cstddef>
#include <algorithm>
#include <vector>
#include <ctime>
#include <cfloat>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/convex_hull_2.h>

#include <queue>
#include <map>

// CGALの型定義
typedef CGAL::Exact_predicates_inexact_constructions_kernel DelaunayK;
typedef CGAL::Triangulation_vertex_base_with_info_2<VertexInfo, DelaunayK> Vb;
typedef CGAL::Triangulation_face_base_2<DelaunayK> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Delaunay_triangulation_2<DelaunayK, Tds> Delaunay;
typedef Delaunay::Point Point_2;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



float TreeSystem::getRandomFloat(float min, float max) {
	return min + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

TreeSystem::TreeSystem()
	: m_RootNode(nullptr), m_pParticleSystem(nullptr),
	m_VAO_Edges(0), m_VBO_Edges(0), m_EdgeVertexCount(0),
	m_VAO_Nodes(0), m_VBO_Nodes(0), m_NodeVertexCount(0),
	m_VAO_Strands(0), m_VBO_Strands(0), m_TotalStrandCount(0),
	m_VAO_Profile(0), m_VBO_Profile(0), m_ProfileVertexCount(64), m_TotalProfileVertexCount(0),
	m_CurrentProfileRadius(0.0f), m_CurrentParticlesPerTip(0),
	m_VAO_StrandCurves(0), m_VBO_StrandCurves(0), m_StrandCurveVertexCount(0),
	m_VAO_Slices(0), m_VBO_Slices(0), m_SliceTotalVertices(0),
	m_VAO_Mesh(0), m_VBO_Mesh(0), m_EBO_Mesh(0), m_MeshIndexCount(0),
	m_VAO_Obstacle(0), m_VBO_Obstacle(0), m_ObstacleVertexCount(0),
	m_VAO_DebugTriangles(0), m_VBO_DebugTriangles(0), m_DebugTriangleVertexCount(0),
	m_NextStrandID(0){
}

TreeSystem::~TreeSystem() {
	if (m_RootNode) delete m_RootNode;
	delete m_pParticleSystem;

	glDeleteVertexArrays(1, &m_VAO_Edges); glDeleteBuffers(1, &m_VBO_Edges);
	glDeleteVertexArrays(1, &m_VAO_Nodes); glDeleteBuffers(1, &m_VBO_Nodes);
	glDeleteVertexArrays(1, &m_VAO_Strands); glDeleteBuffers(1, &m_VBO_Strands);
	glDeleteVertexArrays(1, &m_VAO_Profile); glDeleteBuffers(1, &m_VBO_Profile);
	glDeleteVertexArrays(1, &m_VAO_StrandCurves); glDeleteBuffers(1, &m_VBO_StrandCurves);
	glDeleteVertexArrays(1, &m_VAO_Slices); glDeleteBuffers(1, &m_VBO_Slices);
	glDeleteVertexArrays(1, &m_VAO_Mesh); glDeleteBuffers(1, &m_VBO_Mesh);
	glDeleteVertexArrays(1, &m_VAO_Obstacle); glDeleteBuffers(1, &m_VBO_Obstacle);
	glDeleteVertexArrays(1, &m_VAO_DebugTriangles); glDeleteBuffers(1, &m_VBO_DebugTriangles);
}

void TreeSystem::init() {
	std::srand(static_cast<unsigned int>(std::time(nullptr)));

	// ファイルが存在するか確認
	std::string objPath = OBJ_PATH;
	std::ifstream f(objPath.c_str());

	if (f.good()) {
		f.close();
		// 存在する場合のみロード
		MeshObstacle* meshObs = new MeshObstacle(
			objPath,
			OBJ_SIZE,
			OBJ_POS
		);
		m_pObstacleObj = meshObs;
	}
	else {
		std::cerr << "[WARNING] Obstacle file not found: " << objPath << std::endl;
		std::cerr << "Running without obstacle." << std::endl;
		m_pObstacleObj = nullptr; // 明示的に nullptr にする
	}


	buildTestTree();

	m_pParticleSystem = new ParticleSystem(PROFILE_RADIUS, STRAND_RADIUS);

	// VBO初期化 (すべて空で作成)
	glGenVertexArrays(1, &m_VAO_Edges); glBindVertexArray(m_VAO_Edges);
	glGenBuffers(1, &m_VBO_Edges); glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Edges);
	glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
	glEnableVertexAttribArray(0);

	glGenVertexArrays(1, &m_VAO_Nodes); glBindVertexArray(m_VAO_Nodes);
	glGenBuffers(1, &m_VBO_Nodes); glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Nodes);
	glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
	glEnableVertexAttribArray(0);

	glGenVertexArrays(1, &m_VAO_Strands); glBindVertexArray(m_VAO_Strands);
	glGenBuffers(1, &m_VBO_Strands); glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Strands);
	glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point3D), (void*)offsetof(Point3D, x));
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Point3D), (void*)offsetof(Point3D, r));
	glEnableVertexAttribArray(1);

	glGenVertexArrays(1, &m_VAO_Profile); glBindVertexArray(m_VAO_Profile);
	glGenBuffers(1, &m_VBO_Profile); glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Profile);
	glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
	glEnableVertexAttribArray(0);

	glGenVertexArrays(1, &m_VAO_StrandCurves); glBindVertexArray(m_VAO_StrandCurves);
	glGenBuffers(1, &m_VBO_StrandCurves); glBindBuffer(GL_ARRAY_BUFFER, m_VBO_StrandCurves);
	glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
	glEnableVertexAttribArray(0);

	glGenVertexArrays(1, &m_VAO_Slices); glBindVertexArray(m_VAO_Slices);
	glGenBuffers(1, &m_VBO_Slices); glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Slices);
	glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
	glEnableVertexAttribArray(0);

	glGenVertexArrays(1, &m_VAO_Mesh); glBindVertexArray(m_VAO_Mesh);
	glGenBuffers(1, &m_VBO_Mesh); glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Mesh);
	glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
	glEnableVertexAttribArray(0);
	glGenBuffers(1, &m_EBO_Mesh); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO_Mesh);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

	// デバッグ用VBOの初期化
	glGenVertexArrays(1, &m_VAO_DebugTriangles); glBindVertexArray(m_VAO_DebugTriangles);
	glGenBuffers(1, &m_VBO_DebugTriangles); glBindBuffer(GL_ARRAY_BUFFER, m_VBO_DebugTriangles);
	glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
	glEnableVertexAttribArray(0);
	glBindVertexArray(0);

	//障害物
	if (m_pObstacleObj) {
		//親クラス(Obstacle*)から子クラス(MeshObstacle*)へキャストする
		MeshObstacle* meshPtr = static_cast<MeshObstacle*>(m_pObstacleObj);

		//メッシュから描画用頂点データを取得
		std::vector < glm::vec3 > obsVertices;
		meshPtr->getTriangleVertices(obsVertices); //meshObsではなくmeshPtrをつかう

		// 頂点数を保存 (描画時に使用)
		m_ObstacleVertexCount = static_cast<int>(obsVertices.size());

		glGenVertexArrays(1, &m_VAO_Obstacle); glBindVertexArray(m_VAO_Obstacle);
		glGenBuffers(1, &m_VBO_Obstacle); glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Obstacle);

		// データをGPUへ転送
		glBufferData(GL_ARRAY_BUFFER, obsVertices.size() * sizeof(glm::vec3), obsVertices.data(), GL_STATIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
		glEnableVertexAttribArray(0);
	}
	glBindVertexArray(0);
}


/// --- デバッグ用: ツリー構造を再帰的にログ出力する --- ///
void printTreeRecursive(TreeNode* node, int depth) {
	if (!node) return;

	// インデント作成
	std::string indent(depth * 4, ' ');

	// タイプ名の文字列化
	std::string typeStr;
	switch (node->type) {
	case TreeNode::ROOT:     typeStr = "ROOT"; break;
	case TreeNode::JUNCTION: typeStr = "JUNC"; break;
	case TreeNode::TIP:      typeStr = "TIP "; break;
	default:                 typeStr = "UNKN"; break;
	}

	// 親IDの取得
	int parentID = (node->parent) ? node->parent->id : -1;

	
	// 情報出力
	std::cout << indent
		<< "[" << node->id << "] " << typeStr
		<< " Pos: " << glm::to_string(node->position)
		<< " | Parent: " << parentID
		<< " | Children: " << node->children.size()
		<< std::endl;

	// 子ノードへ再帰
	for (auto child : node->children) {
		printTreeRecursive(child, depth + 1);
	}
}


// CSVファイルから骨格データを読み込む
bool TreeSystem::loadSkeletonFromCSV(const std::string& filepath) {
	std::ifstream file(filepath);
	if (!file.is_open()) {
		std::cerr << "[Error] Failed to open CSV file: " << filepath << std::endl;
		return false;
	}

	// 既存のツリーがあれば削除
	if (m_RootNode) {
		delete m_RootNode;
		m_RootNode = nullptr;
	}

	std::string line;
	// ヘッダー行をスキップ (id,parent_id,x,y,z,thickness)
	std::getline(file, line);

	// 一時的にノードと親IDの関係を保持するマップ
	// Key: ノードID, Value: TreeNodeポインタ
	std::map<int, TreeNode*> nodeMap;
	// Key: ノードID, Value: 親ノードID
	std::map<int, int> parentIdMap;

	int maxId = 0; // 次のID生成用に最大値を記録

	// 1パス目: 全ノードの生成とマップへの登録
	while (std::getline(file, line)) {
		if (line.empty()) continue;

		std::stringstream ss(line);
		std::string token;
		std::vector<std::string> columns;

		// カンマ区切りでパース
		while (std::getline(ss, token, ',')) {
			columns.push_back(token);
		}

		if (columns.size() < 6) continue; // 列不足のエラー処理

		try {
			int id = std::stoi(columns[0]);
			int parentId = std::stoi(columns[1]);
			// 座標と太さをスケーリング
			float x = std::stof(columns[2]) * SKELETON_SCALE;
			float y = std::stof(columns[3]) * SKELETON_SCALE;
			float z = std::stof(columns[4]) * SKELETON_SCALE;
			float thickness = std::stof(columns[5]) * SKELETON_SCALE;

			// タイプは JUNCTION として作成
			// Root候補であっても一旦作成する
			TreeNode* newNode = new TreeNode(id, TreeNode::JUNCTION, glm::vec3(x, y, z), thickness);

			nodeMap[id] = newNode;
			parentIdMap[id] = parentId;

			if (id > maxId) maxId = id;
		}
		catch (const std::exception& e) {
			std::cerr << "[Error] CSV Parse error: " << e.what() << " in line: " << line << std::endl;
		}
	}

	file.close();

	if (nodeMap.empty()) return false;

	// 2パス目: 親子関係のリンク (ポインタ接続)
	for (auto const& [id, node] : nodeMap) {
		int pid = parentIdMap[id];

		if (pid == -1) {
			// 親IDが -1 ならルートノード
			m_RootNode = node;
			node->type = TreeNode::ROOT;
			node->parent = nullptr;
		}
		else {
			// 親を探して接続
			if (nodeMap.find(pid) != nodeMap.end()) {
				TreeNode* parentNode = nodeMap[pid];

				// 親 -> 子
				parentNode->children.push_back(node);
				// 子 -> 親
				node->parent = parentNode;
			}
			else {
				std::cerr << "[Warning] Parent ID " << pid << " not found for Node " << id << std::endl;
			}
		}
	}

	// 3パス目: ノードタイプ(TIP)の確定とクリーンアップ
	// 子を持たないノードを TIP に変更する
	for (auto const& [id, node] : nodeMap) {
		if (node != m_RootNode && node->children.empty()) {
			node->type = TreeNode::TIP;
		}
	}

	// 次に生成される動的ノードのためにIDカウンタを更新
	m_NextStrandID = maxId + 1;

	
	// ログ出力
	std::cout << "Tree loaded from CSV. Total Nodes: " << nodeMap.size() << std::endl;
	if (m_RootNode) {
		//printTreeRecursive(m_RootNode, 0);
	}
	else {
		std::cerr << "[Error] No Root Node (parent_id = -1) found in CSV." << std::endl;
		return false;
	}

	return true;
}


void TreeSystem::buildTestTree() {

	if (m_RootNode) delete m_RootNode;

	// 半径を個別に指定
	float r_root = 0.30f;
	float r_junction = 0.25f;
	float r_mid = 0.2f;
	float r_tip = 0.15f;


	// --- 1. ノードの作成 (ID, タイプ, 座標) ---
	// Level 0: 根
	m_RootNode = new TreeNode(0, TreeNode::ROOT, glm::vec3(0.0f, -1.0f, 0.0f), r_root);
	// Level 1: 最初の分岐点 (Y=1.0)
	TreeNode* junction1 = new TreeNode(1, TreeNode::JUNCTION, glm::vec3(0.0f, 1.0f, 0.0f), r_junction);
	// Level 2: 左側の分岐点 (X=-1.0, Y=2.0) -> 元のTip1の位置を分岐点に変更
	TreeNode* junction2 = new TreeNode(2, TreeNode::JUNCTION, glm::vec3(-1.0f, 2.0f, 0.0f), r_mid);
	// Level 2: 右側の先端 (X=1.0, Y=2.0) -> 元のTip2
	TreeNode* tip3 = new TreeNode(3, TreeNode::TIP, glm::vec3(1.0f, 2.0f, 0.0f), r_tip);
	// Level 3: さらに先の先端 (junction2 から生える)
	TreeNode* tip1 = new TreeNode(4, TreeNode::TIP, glm::vec3(-2.0f, 2.5f, 0.0f), r_tip); // 左へ
	TreeNode* tip2 = new TreeNode(5, TreeNode::TIP, glm::vec3(-1.0f, 3.0f, 0.0f), r_tip); // 右へ
	// --- 2. 親子関係の構築 (ポインタを繋ぐ) ---
	// Root -> Junction1
	junction1->parent = m_RootNode;
	m_RootNode->children.push_back(junction1);
	// Junction1 -> Junction2 (左)
	junction2->parent = junction1;
	junction1->children.push_back(junction2);
	// Junction1 -> Tip3 (右)
	tip3->parent = junction1;
	junction1->children.push_back(tip3);
	// Junction2 -> Tip1 (左-左)
	tip1->parent = junction2;
	junction2->children.push_back(tip1);
	// Junction2 -> Tip2 (左-右)
	tip2->parent = junction2;
	junction2->children.push_back(tip2);
	std::cout << "Complex tree structure built." << std::endl;

	
	//生成直後のツリー情報をログに出す
	std::cout << "================ TREE STRUCTURE LOG ================" << std::endl;
	printTreeRecursive(m_RootNode, 0);
	std::cout << "====================================================" << std::endl;
}


// メイン更新処理 (静的計算版)
void TreeSystem::update(
	float profileRadius, 
	int particlesPerTip, 
	bool obstaclesEnabled
) {

	static bool prevObsEnabled = true;
	bool needsUpdate = false;

	if (m_CurrentProfileRadius != profileRadius ||
		m_CurrentParticlesPerTip != particlesPerTip || //粒子数が変わったとき
		prevObsEnabled != obstaclesEnabled) { //ON・OFFが変わったとき


		// 読み込み試行
		if (!loadSkeletonFromCSV(SKELETON_CSV_PATH)) {
			std::cerr << "CSV loading failed. Fallback to test tree." << std::endl;
			buildTestTree(); // 失敗したらテストツリーに戻す
		}

		//std::cout << "Rebuilt tree from scratch." << std::endl;

		m_CurrentProfileRadius = profileRadius;
		m_CurrentParticlesPerTip = particlesPerTip;
		prevObsEnabled = obstaclesEnabled;
		needsUpdate = true;
	}

	std::vector<glm::vec3> debugTriangleVertices;

	// 障害物フラグが変わったか？ (ON <-> OFF)
	if (prevObsEnabled != obstaclesEnabled) {

		// OFFになった瞬間: 増やしたノードを掃除する
		if (prevObsEnabled && !obstaclesEnabled) {
			PruneGeneratedNodes(m_RootNode);
			//std::cout << "Obstacles disabled: Pruned generated nodes." << std::endl;
		}

		// ONになった瞬間、あるいはOFFになった瞬間、どちらも再描画が必要
		prevObsEnabled = obstaclesEnabled;
		needsUpdate = true;
	}

	if (needsUpdate) {

		// 計測開始
		auto start_time = std::chrono::high_resolution_clock::now();
		std::cout << "[Profile] Start Calculation..." << std::endl;

		m_NextStrandID = 0;


		if (obstaclesEnabled && m_pObstacleObj) {
			RefineTreeRecursive(m_RootNode, m_pObstacleObj, 0);

			RenumberTree();
		}

		// 全ノードのストランド計算 (再帰)
		//再帰処理にフラグを渡す
		processNodeRecursive(m_RootNode, profileRadius, particlesPerTip, obstaclesEnabled);

		// デバッグ用コンテナをクリア
		m_DebugSliceContours.clear();
		m_SliceCounts.clear(); // 以前のデータをクリア

		// VBO更新
		// Strands
		m_AllStrandParticles.clear();
		collectParticlesRecursive(m_RootNode, m_AllStrandParticles);
		m_TotalStrandCount = static_cast<int>(m_AllStrandParticles.size());
		glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Strands);
		glBufferData(GL_ARRAY_BUFFER, m_TotalStrandCount * sizeof(Point3D), m_AllStrandParticles.data(), GL_DYNAMIC_DRAW);

		// Edges
		std::vector<glm::vec3> edges;
		collectEdgesRecursive(m_RootNode, edges);
		m_EdgeVertexCount = static_cast<int>(edges.size());
		glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Edges);
		glBufferData(GL_ARRAY_BUFFER, m_EdgeVertexCount * sizeof(glm::vec3), edges.data(), GL_DYNAMIC_DRAW);

		// --- Nodes (デバッグ用)--- ///
		m_DebugNormalNodes.clear();
		m_DebugGeneratedNodes.clear();
		collectNodesForDebug(m_RootNode);
		m_NodeVertexCount = (int)(m_DebugNormalNodes.size() + m_DebugGeneratedNodes.size());
		glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Nodes);
		glBufferData(GL_ARRAY_BUFFER,
			m_DebugNormalNodes.size() * sizeof(glm::vec3),
			m_DebugNormalNodes.data(),
			GL_DYNAMIC_DRAW);

		// Profiles
		std::vector<glm::vec3> profiles;
		collectProfilesRecursive(m_RootNode, profiles);
		m_TotalProfileVertexCount = static_cast<int>(profiles.size());
		glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Profile);
		glBufferData(GL_ARRAY_BUFFER, m_TotalProfileVertexCount * sizeof(glm::vec3), profiles.data(), GL_DYNAMIC_DRAW);

		//Cureves
		std::vector<glm::vec3> curveVertices;
		generateStrandCurves(curveVertices);
		m_StrandCurveVertexCount = static_cast<int>(curveVertices.size());
		glBindBuffer(GL_ARRAY_BUFFER, m_VBO_StrandCurves);
		glBufferData(GL_ARRAY_BUFFER, m_StrandCurveVertexCount * sizeof(glm::vec3), curveVertices.data(), GL_DYNAMIC_DRAW);

		// Mesh
		m_MeshVertices.clear();
		m_MeshIndices.clear();
		int currentIndex = 0;
		generateMeshRecursive(m_RootNode, m_MeshVertices, m_MeshIndices, currentIndex, obstaclesEnabled);
		m_MeshIndexCount = static_cast<int>(m_MeshIndices.size());
		glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Mesh);
		glBufferData(GL_ARRAY_BUFFER, m_MeshVertices.size() * sizeof(glm::vec3), m_MeshVertices.data(), GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO_Mesh);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_MeshIndices.size() * sizeof(unsigned int), m_MeshIndices.data(), GL_DYNAMIC_DRAW);


		// Slices
		// generateSegmentMeshの中で全ステップの輪郭が m_DebugSliceContours に入っている
		std::vector<glm::vec3> sliceVerticesFlatten;
		for (const auto& loop : m_DebugSliceContours) {
			for (const auto& sv : loop) {
				sliceVerticesFlatten.push_back(sv.pos);
			}
			m_SliceCounts.push_back(static_cast<int>(loop.size()));
		}
		m_SliceTotalVertices = static_cast<int>(sliceVerticesFlatten.size());
		glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Slices);
		glBufferData(GL_ARRAY_BUFFER, m_SliceTotalVertices * sizeof(glm::vec3), sliceVerticesFlatten.data(), GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		//計測終了 & 出力
		auto end_time = std::chrono::high_resolution_clock::now();
		auto duration = end_time - start_time;
		// 各単位に分解して計算
		// 時間
		auto hours = std::chrono::duration_cast<std::chrono::hours>(duration);
		duration -= hours; 
		// 分 
		auto minutes = std::chrono::duration_cast<std::chrono::minutes>(duration);
		duration -= minutes; 
		// 秒 
		auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
		duration -= seconds; 
		// ミリ秒 
		auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);

		// 総ミリ秒数
		double total_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

		//出力
		std::cout << "========================================" << std::endl;
		std::cout << "[Profile] Calculation Finished." << std::endl;

		std::cout << "[Profile] Time: "
			<< hours.count() << "h "
			<< minutes.count() << "m "
			<< seconds.count() << "s "
			<< milliseconds.count() << "ms"
			<< std::endl;

		std::cout << "          (Total: " << total_ms << " ms)" << std::endl;
		std::cout << "========================================" << std::endl;
	}
}



void TreeSystem::RefineTreeRecursive(
	TreeNode* node,
	Obstacle* pObstacle,
	int depth
) {
	if (!node || !pObstacle || depth > 2) return;

	// ダウンキャストして MeshObstacle として扱う
	MeshObstacle* meshObs = dynamic_cast<MeshObstacle*>(pObstacle);

	if (meshObs) {
		glm::vec3 obsMin = meshObs->getMinBounds();
		glm::vec3 obsMax = meshObs->getMaxBounds();

		// 少し余裕を持たせる
		float margin = 0.2f;

		// AABB (軸平行境界ボックス) 判定
		bool inX = (node->position.x >= obsMin.x - margin) && (node->position.x <= obsMax.x + margin);
		bool inY = (node->position.y >= obsMin.y - margin) && (node->position.y <= obsMax.y + margin);
		bool inZ = (node->position.z >= obsMin.z - margin) && (node->position.z <= obsMax.z + margin);

		if (inX && inY && inZ) {
			node->isGenerated = true; // 範囲内ならフラグを立てる
		}
	}

	std::vector<TreeNode*> newChildren;
	bool structureChanged = false;

	for (auto child : node->children) {
		//分割チェック
		//戻り値がnullptr出なければ分割が発生し，新しい先頭ノードが返ってくる
		TreeNode* newHead = CheckAndSubdivide(node, child, pObstacle);

		if (newHead != nullptr) {
			//分割発生
			//新しい鎖の先頭をリストに追加
			newChildren.push_back(newHead);
			//親を自分に設定する
			newHead->parent = node;
			structureChanged = true;
		}
		else {
			//分割なし：そのまま維持
			newChildren.push_back(child);
			//子の先も再帰チェック
			RefineTreeRecursive(child, pObstacle, depth);
		}
	}

	//構造が変わっていたらリストを総入れ替え
	if (structureChanged) {
		node->children = newChildren;
	}
}


// 再帰的にIDを振り直す関数
void TreeSystem::RenumberNodesRecursive(
	TreeNode* node, 
	int& currentID
) {
	if (!node) return;

	// 現在のカウンタ値をIDに設定し、カウントアップ
	node->id = currentID++;

	for (auto child : node->children) {
		RenumberNodesRecursive(child, currentID);
	}
}

void TreeSystem::RenumberTree() {
	// 0番あるいは1番からスタート
	int startID = 0;

	// m_root は TreeSystem が持っているルートノードのメンバ変数と仮定
	if (m_RootNode) {
		RenumberNodesRecursive(m_RootNode, startID);
	}

	// 次に生成されるノードのために、最大IDカウンタも更新しておく
	m_NextStrandID = startID;
}

TreeNode* TreeSystem::CheckAndSubdivide(
	TreeNode* parent,
	TreeNode* child,
	Obstacle* pObstacle
) {
	//大きな木のノードが十分なため
	return nullptr;

	if (!parent || !child || !pObstacle) return nullptr;

	
	// AABBによる大まかな除外 (高速化)
	// メッシュ障害物ならバウンディングボックスを取得
	MeshObstacle* meshObs = dynamic_cast<MeshObstacle*>(pObstacle);
	
	// 干渉判定用のマージン（分割を開始させたい距離）
	float interactionMargin = 0.3f; // 少し広めに

	if (meshObs) {
		glm::vec3 obsMin = meshObs->getMinBounds() - glm::vec3(interactionMargin);
		glm::vec3 obsMax = meshObs->getMaxBounds() + glm::vec3(interactionMargin);

		glm::vec3 segMin = glm::min(parent->position, child->position);
		glm::vec3 segMax = glm::max(parent->position, child->position);

		// 完全に外れているなら即終了
		bool overlap = (segMax.x >= obsMin.x && segMin.x <= obsMax.x) &&
					   (segMax.y >= obsMin.y && segMin.y <= obsMax.y) &&
					   (segMax.z >= obsMin.z && segMin.z <= obsMax.z);
		if (!overlap) return nullptr;
	}

	
	// サンプリングによる精密判定 (傾き・複雑形状に対応)
	// 枝に沿って何点チェックするか
	const int SAMPLE_STEPS = 10; 
	
	float tEnter = 1.0f; // 接触開始 t
	float tExit = 0.0f;  // 接触終了 t
	bool hitDetected = false;

	for (int i = 0; i <= SAMPLE_STEPS; ++i) {
		float t = (float)i / (float)SAMPLE_STEPS;
		glm::vec3 samplePos = glm::mix(parent->position, child->position, t);

		// 障害物までの距離を計測
		float dist;
		glm::vec3 norm;
		pObstacle->getDistanceAndNormal(samplePos, dist, norm);

		// 「内部」または「表面に近い」場合にヒットとみなす
		// distは内部なら負の値、外部なら正の値
		if (dist < interactionMargin) {
			if (!hitDetected) {
				tEnter = t; // 最初のヒット地点
				hitDetected = true;
			}
			tExit = t; // 最後のヒット地点を更新し続ける
		}
	}

	// ヒットしなかったら終了
	if (!hitDetected) return nullptr;

	// 分割範囲の調整 (マージンを持たせる)
	// ヒットした区間を少し広げる
	float tStart = std::max(0.0f, tEnter - 0.1f); // 前に少し広げる
	float tEnd   = std::min(1.0f, tExit + 0.1f);  // 後ろに少し広げる

	// 範囲が狭すぎる場合はノード追加の意味がないので無視
	// (ただし、点接触でも分割したい場合はここを緩める)
	if (tEnd - tStart < 0.05f) {
		// ピンポイントすぎる場合は、その点を中心に少し広げるか、親と子の間を全体的に分割する
		tStart = std::max(0.0f, tStart - 0.1f);
		tEnd = std::min(1.0f, tEnd + 0.1f);
	}

	
	// ノード生成とリンク
	TreeNode* currentParent = parent;
	TreeNode* finalChild = child;
	TreeNode* chainHead = nullptr;

	// 生成するノード数 
	int segments = NODES_SEGMENTS; 

	for (int i = 1; i <= segments; ++i) {
		// tStart から tEnd の間を等分割
		float localT = (float)i / (float)(segments + 1);
		float t = glm::mix(tStart, tEnd, localT);
		
		glm::vec3 newPos = glm::mix(parent->position, child->position, t);

		int newID = m_NextStrandID++;
		TreeNode* newNode = new TreeNode(newID, TreeNode::JUNCTION, newPos);
		newNode->isGenerated = true;

		if (chainHead == nullptr) chainHead = newNode;

		newNode->parent = currentParent;
		if (currentParent && (currentParent != parent)) {
			currentParent->children.push_back(newNode);
		}

		// プロパティ補間
		newNode->u_axis = glm::normalize(glm::mix(parent->u_axis, child->u_axis, t));
		newNode->v_axis = glm::normalize(glm::mix(parent->v_axis, child->v_axis, t));
		newNode->profileRadius = glm::mix(parent->profileRadius, child->profileRadius, t);
		newNode->bundleRadius = glm::mix(parent->bundleRadius, child->bundleRadius, t);

		currentParent = newNode;
	}

	if (currentParent) {
		finalChild->parent = currentParent;
		currentParent->children.push_back(finalChild);
	}

	return chainHead;
}


// 生成されたノード(isGenerated=true)を削除し、
// 親と元の子供を直接つなぎ直す (Pruning / Collapse)
void TreeSystem::PruneGeneratedNodes(TreeNode* node) {
	if (!node) return;

	// 子孫を先に再帰的に掃除する
	for (auto child : node->children) {
		PruneGeneratedNodes(child);
	}

	// 自分の直下の子供リストを整理する
	std::vector<TreeNode*> keptChildren;

	for (auto child : node->children) {
		if (child->isGenerated) {
			// 子が「自動生成ノード」の場合 
			// このノードは削除対象
			//孫を子とする
			
			for (auto grandChild : child->children) {
				// 孫の親を「自分」に書き換え
				grandChild->parent = node;
				keptChildren.push_back(grandChild);
			}

			// 生成ノードは用済みなのでメモリ解放
			delete child;
		}
		else {
			// 子が「オリジナルノード」の場合
			// そのまま維持
			keptChildren.push_back(child);
		}
	}

	// 子供リストを更新
	node->children = keptChildren;
}

// 比較用の関数
bool compareChildrenByParticleCount(TreeNode* a, TreeNode* b) {
	return a->strandParticles.size() > b->strandParticles.size();
}


// 再帰的処理
void TreeSystem::processNodeRecursive(
	TreeNode* node,
	float baseProfileRadius,
	int particlesPerTip,
	bool obstaclesEnabled
) {

	glm::vec3 tangent;
	if (node->parent) {
		tangent = glm::normalize(node->position - node->parent->position);

		//親の軸を引き継ぐ (Parallel Transport)
		// 親の接線ベクトル
		glm::vec3 parentTangent;
		if (node->parent->parent) {
			parentTangent = glm::normalize(node->parent->position - node->parent->parent->position);
		}
		else {
			// 親がルートの場合、Y軸などを仮定
			parentTangent = glm::vec3(0.0f, 1.0f, 0.0f);
			// もし親のu_axisが未定義なら初期化
			if (glm::length(node->parent->u_axis) < 0.001f) {
				getOrthonormalBasis(parentTangent, node->parent->u_axis, node->parent->v_axis);
			}
		}

		// 回転の計算 (親の向き -> 自分の向き)
		// 2つのベクトルの間の「最短回転」を求める
		float dot = glm::dot(parentTangent, tangent);
		glm::vec3 cross = glm::cross(parentTangent, tangent);

		if (glm::length(cross) < 0.0001f) {
			// 平行（直線）なら、軸をそのままコピー
			node->u_axis = node->parent->u_axis;
		}
		else {
			// 回転軸と角度
			float angle = std::acos(glm::clamp(dot, -1.0f, 1.0f));
			glm::vec3 axis = glm::normalize(cross);

			// 親のu_axisを回転させて、子のu_axisにする
			// (GLMの回転行列を使用)
			glm::mat4 rotMat = glm::rotate(glm::mat4(1.0f), angle, axis);
			node->u_axis = glm::vec3(rotMat * glm::vec4(node->parent->u_axis, 0.0f));
		}

		// 誤差蓄積を防ぐために正規化し、v_axisは外積で再計算
		node->u_axis = glm::normalize(node->u_axis);
		node->v_axis = glm::normalize(glm::cross(tangent, node->u_axis));

	}
	else {
		// ルートノードは基準を作る
		tangent = glm::vec3(0.0f, 1.0f, 0.0f);
		if (!node->children.empty()) {
			tangent = glm::normalize(node->children[0]->position - node->position);
		}
		getOrthonormalBasis(tangent, node->u_axis, node->v_axis);
	}

	//子ノードの再帰処理(軸計算の後に呼ぶ)
	for (auto child : node->children) {
		processNodeRecursive(child, baseProfileRadius, particlesPerTip, obstaclesEnabled);
	}

	//ストランド生成と転送処理
	node->strandParticles.clear();
	node->refStrandParticles.clear();

	if (node->type == TreeNode::TIP) {

		float r = node->profileRadius;

		for (int i = 0; i < particlesPerTip; ++i) {
			const float goldenAngle = 2.39996323f; // rad

			float angle = i * goldenAngle;
			float t = (i + 0.5f) / particlesPerTip;
			float dist = r * std::sqrt(t);


			// 3D位置
			glm::vec3 local =
				(dist * std::cos(angle) * node->u_axis) +
				(dist * std::sin(angle) * node->v_axis);

			glm::vec3 pos = node->position + local;

			Point3D p = {};
			p.x = pos.x; p.y = pos.y; p.z = pos.z;
			p.id = m_NextStrandID++;

			if (node->id == 2) { 
				p.r = 1.0f; 
				p.g = 1.0f; 
			}
			else { 
				p.g = 1.0f; 
				p.b = 1.0f; 
			}

			node->strandParticles.push_back(p);
			node->refStrandParticles.push_back(p);
		}
		node->bundleRadius = r;
	}
	else {
		std::sort(node->children.begin(), node->children.end(), compareChildrenByParticleCount);

		if (node->children.empty()) return;

		m_pParticleSystem->setProfileRadius(node->profileRadius);
		m_pParticleSystem->clearParticles();

		//障害物をセット
		//UIがONかつオブジェクトが存在する場合のみ障害物をセット
		if (obstaclesEnabled && m_pObstacleObj) {
			m_pParticleSystem->setObstacle(m_pObstacleObj);
		}
		else {
			//Offならnullptrをセット(PBDで無視される)
			m_pParticleSystem->setObstacle(nullptr);
		}

		//ノードの3Dコンテキスト(原点,U,V)をセット
		//これによりParticleSystemないで 2D <-> 3D 変換が可能になる
		m_pParticleSystem->setNodeContext(node->position, node->u_axis, node->v_axis);

		//PBD計算用の初期状態リスト
		std::vector<Point3D> tempParticles;

		for (size_t c = 0; c < node->children.size(); ++c) {
			TreeNode* directChild = node->children[c]; //直下の子

			//ソースノードの決定ロジック
			TreeNode* sourceNode = directChild;

			//自分がオリジナルかつ子が生成ノードならその先のオリジナルを探す
			if (!node->isGenerated && directChild->isGenerated) {
				TreeNode* temp = directChild;
				//生成ノードである限り，子をたどる
				while (temp && temp->isGenerated && !temp->children.empty()) {
					temp = temp->children[0];
				}
				//もしたどり着いた先がオリジナルなら，それをソースにする
				if (temp && !temp->isGenerated) {
					sourceNode = temp;
				}
			}

			//以下directChildではなくsourceNodeの情報を使う

			glm::vec2 displacement(0.0f);
			if (c > 0) {//小枝を処理するとき
				TreeNode* mainSource = node->children[0];

				float D_large = mainSource->bundleRadius;
				float D_small = sourceNode->bundleRadius; // sourceNodeを使用
				float d_i = D_large + D_small;

				//ベクトル計算は直下の子との位置関係で行う
				glm::vec3 childDir = directChild->position - node->position;
				glm::vec2 v;
				v.x = glm::dot(childDir, node->u_axis);
				v.y = glm::dot(childDir, node->v_axis);
				glm::vec2 v_norm(0.0f);
				if (glm::length(v) > 0.0001f) v_norm = glm::normalize(v);
				displacement = v_norm * d_i; // これを追加
			}

			//投影元データ：sourceNodeのstrandParticles(PBD結果)を使う
			//これにより中間ノードがつぶれていても，オリジナルの太さが維持される
			const std::vector<Point3D>& sourceParticles = sourceNode->strandParticles;

			for (const auto& sp : sourceParticles) {
				// sourceNode での (u, v) を取り出し、それをそのまま node に適用する
				glm::vec3 vec_from_source_center = glm::vec3(sp.x, sp.y, sp.z) - sourceNode->position;

				float source_u = glm::dot(vec_from_source_center, sourceNode->u_axis);
				float source_v = glm::dot(vec_from_source_center, sourceNode->v_axis);

				glm::vec2 p_i(source_u, source_v);
				glm::vec2 p_prime_i = p_i + displacement;

				int currentID = static_cast<int>(tempParticles.size());
				m_pParticleSystem->addParticle(p_prime_i, currentID);
				tempParticles.push_back(sp); // ID等の属性引継ぎ

				// 3D位置への復元
				glm::vec3 p3d_ref = node->position + (p_prime_i.x * node->u_axis) + (p_prime_i.y * node->v_axis);
				Point3D refP = sp;
				refP.x = p3d_ref.x; refP.y = p3d_ref.y; refP.z = p3d_ref.z;
				node->refStrandParticles.push_back(refP);
			}
		}


		// PBD実行
		m_pParticleSystem->solve(SOLVE);

		const auto& final_particles = m_pParticleSystem->getParticles();

		float maxDistSq = 0.0f;

		for (const auto& fp : final_particles) {

			int originalIndex = fp.id;
			Point3D originalData = tempParticles[originalIndex];
			//計算結果の2D位置
			glm::vec2 p2d = fp.position; //fp.positionにはp_prime_iが入っている
			//3D位置への復元
			glm::vec3 p3d = node->position + (p2d.x * node->u_axis) + (p2d.y * node->v_axis);

			//色データなどは元データから
			Point3D newPoint = originalData;
			newPoint.x = p3d.x;
			newPoint.y = p3d.y;
			newPoint.z = p3d.z;

			//色の変更ロジック
			if (fp.initially_inside) {
				newPoint.r = 1.0f;
				newPoint.g = 0.6f;
				newPoint.b = 0.0f;
			}
			else {
				newPoint.r = 0.0f;
				newPoint.g = 1.0f;
				newPoint.b = 0.0f;
			}

			node->strandParticles.push_back(newPoint);

			float distSq = glm::dot(p2d, p2d);
			if (distSq > maxDistSq) maxDistSq = distSq;
		}

		node->bundleRadius = std::sqrt(maxDistSq);
	}
}


void TreeSystem::collectParticlesRecursive(TreeNode* node, std::vector<Point3D>& outParticles) {
	outParticles.insert(outParticles.end(), node->strandParticles.begin(), node->strandParticles.end());
	for (auto child : node->children) collectParticlesRecursive(child, outParticles);
}

void TreeSystem::collectEdgesRecursive(TreeNode* node, std::vector<glm::vec3>& outVertices) {
	if (node->parent) {
		outVertices.push_back(node->parent->position);
		outVertices.push_back(node->position);
	}
	for (auto child : node->children) collectEdgesRecursive(child, outVertices);
}

void TreeSystem::collectProfilesRecursive(TreeNode* node, std::vector<glm::vec3>& outVertices) {
	float currentRadius = node->profileRadius;
	for (int i = 0; i < m_ProfileVertexCount; ++i) {
		float angle = 2.0f * (float)M_PI * (float)i / (float)m_ProfileVertexCount;
		float x = currentRadius * glm::cos(angle);
		float y = currentRadius * glm::sin(angle);
		glm::vec3 pos = node->position + (x * node->u_axis) + (y * node->v_axis);
		outVertices.push_back(pos);
	}
	for (auto child : node->children) collectProfilesRecursive(child, outVertices);
}

void TreeSystem::collectNodesForDebug(TreeNode* node) {
	if (!node) return;

	if (node->isGenerated) {
		m_DebugGeneratedNodes.push_back(node->position);
	}
	else {
		m_DebugNormalNodes.push_back(node->position);
	}
	for (auto child : node->children) {
		collectNodesForDebug(child);
	}
}

void TreeSystem::generateMeshRecursive(
	TreeNode* node,
	std::vector<glm::vec3>& meshVertices,
	std::vector<unsigned int>& meshIndices,
	int& currentIndex,
	bool obstaclesEnabled
) {
	if (!node) return;

	// 子ノードとの間にメッシュを生成
	for (auto child : node->children) {
		generateSegmentMesh(node, child, meshVertices, meshIndices, currentIndex, obstaclesEnabled);

		// 子ノードへ再帰
		generateMeshRecursive(child, meshVertices, meshIndices, currentIndex, obstaclesEnabled);
	}
}

void TreeSystem::collectTipsRecursive(
	TreeNode* node, 
	std::vector<TreeNode*>& outTips
){
	if (!node) return;

	if (node->type == TreeNode::TIP) {
		outTips.push_back(node);
	}

	for (auto child : node->children) {
		collectTipsRecursive(child, outTips);
	}
}
//ストランドジオメトリ
void TreeSystem::generateStrandCurves(
	std::vector<glm::vec3>& outCurveVertices
){ 
	outCurveVertices.clear();
	std::vector<TreeNode*> tips;

	// Tip探索
	collectTipsRecursive(m_RootNode, tips);

	// 各Tipから根元へのストランドを追跡
	for (TreeNode* tip : tips) {
		for (int i = 0; i < tip->strandParticles.size(); ++i) {
			std::vector<glm::vec3> controlPoints;

			// --- 制御点の収集  ---
			TreeNode* currentNode = tip;
			int currentIndex = i;
			while (currentNode != nullptr) {
				if (currentIndex < currentNode->strandParticles.size()) {
					Point3D p = currentNode->strandParticles[currentIndex];
					controlPoints.push_back(glm::vec3(p.x, p.y, p.z));
				}

				// 親へ遡る処理 
				TreeNode* parent = currentNode->parent;
				if (parent) {
					int siblingOffset = 0;
					for (auto sibling : parent->children) {
						if (sibling == currentNode) break;
						siblingOffset += sibling->strandParticles.size();
					}
					currentIndex = siblingOffset + currentIndex;
				}
				currentNode = parent;
			}

			//Bスプライン曲線
			CurveGenerator::generateBSpline(controlPoints, outCurveVertices);
			//ファガーソン曲線
			//CurveGenerator::generateFerguson(controlPoints, curveVertices);
		}
	}
}


glm::vec3 ComputeCentroid(const std::vector<SliceVertex>& loop)
{
	glm::vec3 c(0.0f);
	for (const auto& v : loop) c += v.pos;
	return c / (float)loop.size();
}


// 2D平面上の点包含判定 (Ray Casting Algorithm)
// p: 判定したい点 (u, v)
// loop: 多角形の頂点リスト
// center, u_axis, v_axis: 3D->2D投影用
bool IsPointInPolygon(
	const glm::vec3& p3D,
	const std::vector<SliceVertex>& loop,
	const glm::vec3& center,
	const glm::vec3& u_axis,
	const glm::vec3& v_axis
) {
	// 3D -> 2D 投影
	auto to2D = [&](const glm::vec3& pos) {
		glm::vec3 rel = pos - center;
		return glm::vec2(glm::dot(rel, u_axis), glm::dot(rel, v_axis));
		};

	glm::vec2 p = to2D(p3D);
	bool inside = false;

	// レイキャスティング法
	for (size_t i = 0, j = loop.size() - 1; i < loop.size(); j = i++) {
		glm::vec2 pi = to2D(loop[i].pos);
		glm::vec2 pj = to2D(loop[j].pos);

		if (((pi.y > p.y) != (pj.y > p.y)) &&
			(p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x)) {
			inside = !inside;
		}
	}
	return inside;
}

// 符号付き面積を計算する関数
// 戻り値:
//   正 (+) : 反時計回り (CCW) -> 外殻 (Outer)
//   負 (-) : 時計回り (CW)   -> 穴 (Hole)
float TreeSystem::ComputeSignedArea(
	const std::vector<SliceVertex>& loop,
	const glm::vec3& centerPos,
	const glm::vec3& u_axis,
	const glm::vec3& v_axis
) {
	// 頂点が3つ未満なら面積なし
	if (loop.size() < 3) return 0.0f;

	float area = 0.0f;

	for (size_t i = 0; i < loop.size(); ++i) {
		// 現在の点 (P1) と 次の点 (P2) を取得 (最後の次は最初に戻る)
		const glm::vec3& p1_3d = loop[i].pos;
		const glm::vec3& p2_3d = loop[(i + 1) % loop.size()].pos;

		// 3D座標 -> 2Dローカル座標 (u, v) への投影
		//    相対ベクトルを計算
		glm::vec3 rel1 = p1_3d - centerPos;
		glm::vec3 rel2 = p2_3d - centerPos;

		//    内積で u, v 成分を抽出
		float u1 = glm::dot(rel1, u_axis);
		float v1 = glm::dot(rel1, v_axis);
		float u2 = glm::dot(rel2, u_axis);
		float v2 = glm::dot(rel2, v_axis);

		// 外積成分 (u1*v2 - u2*v1) を加算
		area += (u1 * v2 - u2 * v1);
	}

	// 最後に 1/2 する
	return area * 0.5f;
}

int getVertexHandleID(
	Delaunay::Vertex_handle vh,
	std::map<Delaunay::Vertex_handle, int>& vhToId,
	std::vector<Delaunay::Vertex_handle>& idToVh)
{
	// 新しいIDを作って登録
	if (vhToId.find(vh) == vhToId.end()) {
		int id = (int)idToVh.size();
		vhToId[vh] = id;
		idToVh.push_back(vh);
		return id;
	}
	// 既に登録されていれば、そのIDを返す
	return vhToId[vh];
}


// 点群を受け取り、閾値 α に基づいて「境界ループのリスト」を返す
// 戻り値: vector<vector<vec2>> (分岐した場合、複数のループが返る)
void TreeSystem::getAlphaShapeContours(
	const std::vector<SliceVertex>& points,
	float threshold,
	Obstacle* pObstacle,
	const glm::vec3& centerPos,
	const glm::vec3& u_axis,
	const glm::vec3& v_axis,
	std::vector<Contour>& outContours,
	bool obstaclesEnabled
) {
	outContours.clear();

	if (points.size() < 3) return;

	// 投影とドロネー分割
	Delaunay dt;
	for (const auto& p : points) {

		// 3D座標を2D平面(u, v)に投影してCGALに入れる
		glm::vec3 relative = p.pos - centerPos;
		double u = glm::dot(relative, u_axis);
		double v = glm::dot(relative, v_axis);

		//頂点を挿入し、情報をセット
		auto vh = dt.insert(Point_2(u, v));
		vh->info().strandID = p.strandID;
	}

	// 三角形のフィルタリング 


	// デバッグ用カウンタ
	int totalFaces = 0;       // 全三角形数
	int removedByEdge = 0;    // 辺が長すぎて消された数
	int removedByObs = 0;     // 障害物に接触して消された数
	int keptFaces = 0;        // 残った数
	// マージン設定
	float removalMargin = REMOVAL_MAEGIN;

	float thresholdSq = threshold * threshold;
	std::vector<Delaunay::Face_handle> validFaces;
	std::map<Delaunay::Face_handle, bool> visited;
	// フェースの状態を記録するマップ
	// 0: 有効 (Valid)
	// 1: 辺長制限で削除 (EdgeKill)
	// 2: 障害物で削除 (ObsKill) -> これが隣にあると InnerArc
	std::map<Delaunay::Face_handle, int> faceStatus;


	// 障害物のAABB情報を事前に取得
	glm::vec3 obsMin(0), obsMax(0);
	MeshObstacle* meshObs = nullptr;
	if (pObstacle) {
		meshObs = dynamic_cast<MeshObstacle*>(pObstacle);
		if (meshObs) {
			obsMin = meshObs->getMinBounds();
			obsMax = meshObs->getMaxBounds();
		}
	}


	for (auto fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
		totalFaces++; //ログ用ポインタ
		Point_2 p0 = fit->vertex(0)->point();
		Point_2 p1 = fit->vertex(1)->point();
		Point_2 p2 = fit->vertex(2)->point();

		// 閾値チェック (分岐判定用)
		float d1 = CGAL::squared_distance(p0, p1);
		float d2 = CGAL::squared_distance(p1, p2);
		float d3 = CGAL::squared_distance(p2, p0);

		if (d1 > thresholdSq || d2 > thresholdSq || d3 > thresholdSq) {
			faceStatus[fit] = 1; // EdgeKill
			removedByEdge++; 
			continue; // エッジが長すぎるので削除
		}

		//  障害物チェック (穴あけ用)
		if (pObstacle) {
			// 三角形の重心を計算 (2D)
			float cx = (float)(p0.x() + p1.x() + p2.x()) / 3.0f;
			float cy = (float)(p0.y() + p1.y() + p2.y()) / 3.0f;

			// 重心を3Dワールド座標に変換
			glm::vec3 centroid3D = centerPos + (cx * u_axis) + (cy * v_axis);

			bool performCheck = true;
			if (meshObs) {
				// マージンを含めたAABB内にあるか？
				float m = removalMargin + 0.1f; // 少し余裕を持たせる
				if (centroid3D.x < obsMin.x - m || centroid3D.x > obsMax.x + m ||
					centroid3D.y < obsMin.y - m || centroid3D.y > obsMax.y + m ||
					centroid3D.z < obsMin.z - m || centroid3D.z > obsMax.z + m) {
					performCheck = false; // 箱の外なら計算しない
				}
			}
			
			// 箱の中にいる可能性がある時だけ、重い計算(getDistanceAndNormal)をする
			if (performCheck) {

				// 障害物との距離を判定
				float dist;
				glm::vec3 normal;
				pObstacle->getDistanceAndNormal(centroid3D, dist, normal);

				if (dist < removalMargin) {
					faceStatus[fit] = 2;
					removedByObs++; // 
					//実際に消された場所の距離をログに出す場合
					//if (removedByObs <= 5) std::cout << "  [ObsKill] dist: " << dist << std::endl;
					continue; // 障害物の中(または近傍)なので削除
				}
			}
		}

		// 合格した三角形のみ登録
		validFaces.push_back(fit);
		faceStatus[fit] = 0; // Valid
		visited[fit] = false;
		keptFaces++; 
	}

	if (validFaces.empty()) return;

	//連結成分ごとにグループ化し、境界線を抽出
	for (auto startFace : validFaces) {
		if (visited[startFace]) continue;

		// 既存のエッジマップ構築ロジックの中で、頂点ハンドル(Vertex_handle)をキーにする
		std::map<Delaunay::Vertex_handle, Delaunay::Vertex_handle> edgeMapVH;

		// BFSで連結成分を集める
		std::vector<Delaunay::Face_handle> componentFaces;
		std::queue<Delaunay::Face_handle> q;

		q.push(startFace);
		visited[startFace] = true;
		componentFaces.push_back(startFace);

		// 成分内のFace検索用のSet (高速検索用)
		std::set<Delaunay::Face_handle> componentSet;
		componentSet.insert(startFace);

		while (!q.empty()) {
			auto current = q.front();
			q.pop();

			for (int i = 0; i < 3; ++i) {
				auto neighbor = current->neighbor(i);
				// neighborが「有効リスト」に含まれていて、かつ未訪問なら
				if (visited.find(neighbor) != visited.end() && !visited[neighbor]) {
					visited[neighbor] = true;
					q.push(neighbor);
					componentFaces.push_back(neighbor);
					componentSet.insert(neighbor);
				}
			}
		}

		// 境界辺の抽出とループ構築
		std::map<int, int> edgeMap;
		std::map<Delaunay::Vertex_handle, int> vhToId;
		std::vector<Delaunay::Vertex_handle> idToVh;

		//境界辺の収集
		for (auto fh : componentFaces) {//ここはBFSで集めたFace
			for (int i = 0; i < 3; ++i) {
				auto neighbor = fh->neighbor(i);
				// 隣が「無限」または「成分外」なら、その辺(i)は境界線
				bool is_boundary = (dt.is_infinite(neighbor) ||
					componentSet.find(neighbor) == componentSet.end());

				if (is_boundary) {
					auto vA = fh->vertex((i + 1) % 3);
					auto vB = fh->vertex((i + 2) % 3);
					
					int idA = getVertexHandleID(vA, vhToId, idToVh);
					int idB = getVertexHandleID(vB, vhToId, idToVh);
					edgeMap[idA] = idB;

					bool isInnerEdge = (!dt.is_infinite(neighbor) && faceStatus[neighbor] == 2);
					// 頂点情報に書き込むための準備
					if (isInnerEdge) {
						// vh->info() は VertexInfo なので、そこに一時的にフラグを立てる手もある
						vA->info().isInnerCandidate = true;
						vB->info().isInnerCandidate = true;
					}
				}
			}
		}

		struct RawLoop {
			std::vector<SliceVertex> verts;
			float area;     // 符号付き面積
			float absArea;  // 絶対値
			glm::vec3 center; // 重心
		};
		std::vector<RawLoop> rawLoops;


		// エッジマップからループを構築 (一筆書き)
		//一筆書きedgeMapをたどって閉じたループ(頂点リスト)を作成
		while (!edgeMap.empty()) {
			std::vector<SliceVertex> loopVertices; //生の頂点リスト

			// 適当な始点を取り出す
			auto it = edgeMap.begin();
			int startID = it->first;
			int currID = startID;

			// ループをたどる
			bool closed = false;
			while (edgeMap.find(currID) != edgeMap.end()) {
				//情報を復元して保存
				auto vh = idToVh[currID];
				Point_2 p = vh->point();
				int sID = vh->info().strandID;
				bool isInner = vh->info().isInnerCandidate;


				//2D->3D復元
				glm::vec3 pos3D = centerPos + ((float)p.x() * u_axis) + ((float)p.y() * v_axis);

				loopVertices.push_back({ pos3D, sID, isInner });

				int nextID = edgeMap[currID];
				edgeMap.erase(currID); // 使った辺は消す
				currID = nextID;

				if (currID == startID) {
					closed = true;
					break;
				}
			}

			// 閉じたループができたら登録
			if (closed && loopVertices.size() >= 3) {

				RawLoop rl;
				rl.verts = loopVertices;

				// 面積と重心を計算
				float signedArea = ComputeSignedArea(loopVertices, centerPos, u_axis, v_axis);
				rl.area = signedArea;
				rl.absArea = std::abs(signedArea);
				rl.center = ComputeCentroid(loopVertices);

				// すべて一旦「反時計回り(CCW)」に正規化しておく
				// 後の包含判定や、Outer/Holeへの割り当て時に、必要に応じて反転させるため
				if (signedArea < 0) {
					std::reverse(rl.verts.begin(), rl.verts.end());
				}

				rawLoops.push_back(rl);
			}
		}

		if (rawLoops.empty()) return;

		// 面積の大きい順にソート
		std::sort(rawLoops.begin(), rawLoops.end(), [](const RawLoop& a, const RawLoop& b) {
			return a.absArea > b.absArea;
			});
		// --- 包含関係の解決 ---
			// 大きい順に見ていく
			// まだ処理されていないループのうち、自分が「内包している」ものがあれば、それは自分の「穴」である
			// 自分を内包するものがなければ、自分は新しい「島(Outer)」である

		std::vector<bool> isConsumed(rawLoops.size(), false);

		for (size_t i = 0; i < rawLoops.size(); ++i) {
			if (isConsumed[i]) continue;

			// 新しい Contour (島) を作成
			Contour island;
			island.outer = rawLoops[i].verts; // OuterはCCW (正規化済み)
			island.centroid = rawLoops[i].center;
			isConsumed[i] = true;

			// 自分より小さいループたちをチェック
			for (size_t j = i + 1; j < rawLoops.size(); ++j) {
				if (isConsumed[j]) continue;

				// ループJ の重心が ループI の中にあるか？
				if (IsPointInPolygon(rawLoops[j].center, island.outer, centerPos, u_axis, v_axis)) {

					// J は I の穴である
					std::vector<SliceVertex> holeLoop = rawLoops[j].verts;

					// 穴は「時計回り(CW)」にする
					std::reverse(holeLoop.begin(), holeLoop.end());

					island.holes.push_back(holeLoop);
					isConsumed[j] = true;
				}
			}

			outContours.push_back(island);
		}
	}

}

// 4点とt(0~1)からBスプライン上の位置を計算
glm::vec3 evalBSpline(
	const glm::vec3& p0, 
	const glm::vec3& p1, 
	const glm::vec3& p2, 
	const glm::vec3& p3, float t
) {
	float t2 = t * t;
	float t3 = t2 * t;

	// B-Spline 基底関数 (Uniform Cubic)
	// 行列:
	// [-1  3 -3  1]
	// [ 3 -6  3  0]
	// [-3  0  3  0]
	// [ 1  4  1  0] / 6.0

	float b0 = (-t3 + 3.0f * t2 - 3.0f * t + 1.0f) / 6.0f;
	float b1 = (3.0f * t3 - 6.0f * t2 + 4.0f) / 6.0f;
	float b2 = (-3.0f * t3 + 3.0f * t2 + 3.0f * t + 1.0f) / 6.0f;
	float b3 = t3 / 6.0f;

	return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

// 4点とt(0~1)からCatmull-Romスプライン上の位置を計算
// P1(t=0) から P2(t=1) への曲線を生成します
glm::vec3 evalCatmullRom(
	const glm::vec3& p0, 
	const glm::vec3& p1, 
	const glm::vec3& p2, 
	const glm::vec3& p3, float t
) {
	float t2 = t * t;
	float t3 = t2 * t;

	return 0.5f * (
		(2.0f * p1) +
		(-p0 + p2) * t +
		(2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
		(-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3
		);
}


//Bスプライン補間版
//catmull-romスプライン版
void TreeSystem::InterpolateStrands(
	TreeNode* parent,
	TreeNode* child,
	float t,
	std::vector<SliceVertex>& outStrands,
	glm::vec3& outCenter,
	glm::vec3& outU,
	glm::vec3& outV
) {
	outStrands.clear();

	// 制御点となるノードの特定 (P0 -> P1(Parent) -> P2(Child) -> P3)
	TreeNode* grandParent = parent->parent;

	// P3 (子の先) を探す
	// 子がTIPなら先はないので、直線的に延長する仮想点を考える
	// 分岐がある場合、ストランドがどの枝へ続くか不明確だが、
	// ここでは代表として「一番太い子」または「最初の子」をP3とする（簡易実装）
	TreeNode* grandChild = nullptr;
	if (!child->children.empty()) {
		grandChild = child->children[0]; // 主枝を採用
	}

	
	// 基準座標系(Center) の Bスプライン補間
	{
		glm::vec3 p1 = parent->position;
		glm::vec3 p2 = child->position;

		// 端点の処理（存在しない場合は直線的に延長した位置を仮想P0/P3とする）
		glm::vec3 p0 = (grandParent) ? grandParent->position : (p1 + (p1 - p2));
		glm::vec3 p3 = (grandChild) ? grandChild->position : (p2 + (p2 - p1));

		// 中心位置をスプラインで動かす
		outCenter = evalCatmullRom(p0, p1, p2, p3, t);
	}

	outU = glm::normalize(glm::mix(parent->u_axis, child->u_axis, t));
	glm::vec3 tempV = glm::normalize(glm::mix(parent->v_axis, child->v_axis, t));
	glm::vec3 tangent = glm::normalize(glm::cross(outU, tempV));
	outV = glm::normalize(glm::cross(tangent, outU));

	// ストランド粒子の Bスプライン補間
	for (const auto& childP : child->strandParticles) {

		// P2: 現在の子の粒子位置
		glm::vec3 posP2 = glm::vec3(childP.x, childP.y, childP.z);

		// P1: 親の粒子位置 (ID検索)
		glm::vec3 posP1 = posP2; // 見つからない場合のフォールバック
		bool foundP1 = false;
		for (const auto& pp : parent->strandParticles) {
			if (pp.id == childP.id) {
				posP1 = glm::vec3(pp.x, pp.y, pp.z);
				foundP1 = true;
				break;
			}
		}

		// P0: 親の親の粒子位置 (ID検索)
		glm::vec3 posP0 = posP1 + (posP1 - posP2); // デフォルト: P2->P1の延長
		if (grandParent && foundP1) {
			for (const auto& gpp : grandParent->strandParticles) {
				if (gpp.id == childP.id) {
					posP0 = glm::vec3(gpp.x, gpp.y, gpp.z);
					break;
				}
			}
		}

		// P3: 子の先の粒子位置 (ID検索)
		// 子の子供たちを走査して、同じストランドIDを持つものを探す
		glm::vec3 posP3 = posP2 + (posP2 - posP1); // デフォルト: P1->P2の延長
		if (!child->children.empty()) {
			bool foundP3 = false;
			for (auto gc : child->children) {
				for (const auto& gcp : gc->strandParticles) {
					if (gcp.id == childP.id) {
						posP3 = glm::vec3(gcp.x, gcp.y, gcp.z);
						foundP3 = true;
						break;
					}
				}
				if (foundP3) break; // 見つかったら終了
			}
		}

		// Bスプライン補間を実行
		SliceVertex sv;
		sv.pos = evalCatmullRom(posP0, posP1, posP2, posP3, t);
		sv.strandID = childP.id;

		outStrands.push_back(sv);
	}
}

//ヘルパー：ループBの中でターゲットに最も近い頂点のインデックスを返す
int TreeSystem::FindBestMatchIndex(
	const std::vector<SliceVertex>& loop,
	int targetID,
	const glm::vec3& targetPos
){
	int bestIndex = -1;
	float minSqDist = FLT_MAX;

	// IDで完全一致を探す
	for (int i = 0; i < loop.size(); ++i) {
		if (loop[i].strandID == targetID) {
			return i;
		}
	}

	// IDが見つからない場合 (消滅した場合など) は、距離で探す
	for (int i = 0; i < loop.size(); ++i) {
		float sqDist = glm::length2(loop[i].pos - targetPos);
		if (sqDist < minSqDist) {
			minSqDist = sqDist;
			bestIndex = i;
		}
	}
	return bestIndex;
}

// ヘルパー: 2つの輪郭をジッパーのように縫い合わせる
void TreeSystem::StitchZipper(
	const std::vector<SliceVertex>& loopA, // Prev
	const std::vector<SliceVertex>& loopB, // Curr
	std::vector<glm::vec3>& meshVertices,
	std::vector<unsigned int>& meshIndices,
	int& currentIndex
) {
	if (loopA.empty() || loopB.empty()) return;

	int startB = -1;
	bool matchFound = false;

	// 全探索で「共通のID」を探す
	for (int i = 0; i < loopA.size(); ++i) {
		int idA = loopA[i].strandID;
		if (idA == -1) continue; // IDなしはスキップ

		// loopBの中から idA を探す
		for (int j = 0; j < loopB.size(); ++j) {
			if (loopB[j].strandID == idA) {
				// 共通ID(A[i] と B[j] が同じ)
				// Aのスタート(0)は、Bの (j - i) の位置に対応するはず

				// 負の数にならないようにループサイズを足して余りをとる
				int sizeB = (int)loopB.size();
				startB = (j - i % sizeB + sizeB) % sizeB;

				matchFound = true;

				break;
			}
		}
		if (matchFound) break; // 1つ見つかれば十分
	}

	// もし共通IDが一つもなければ、距離ベース
	if (!matchFound) {
		//std::cout << "    [ZIPPER WARNING] No common ID found! Fallback to closest point." << std::endl;
		startB = FindBestMatchIndex(loopB, loopA[0].strandID, loopA[0].pos);
	}

	if (startB == -1) startB = 0;

	int N_A = (int)loopA.size();
	int N_B = (int)loopB.size();

	int i = 0; // LoopA のローカル進行度
	int j = 0; // LoopB のローカル進行度

	// ループ一周分の処理が終わるまで回す
	while (i < N_A || j < N_B) {

		// 現在の頂点インデックス
		int i_curr = i % N_A;
		int i_next = (i + 1) % N_A;
		int j_curr = (startB + j) % N_B;
		int j_next = (startB + j + 1) % N_B;

		// 頂点データ
		const auto& vA_curr = loopA[i_curr];
		const auto& vA_next = loopA[i_next];
		const auto& vB_curr = loopB[j_curr];
		const auto& vB_next = loopB[j_next];

		bool advanceA = false;

		if (i >= N_A) {
			advanceA = false; // A完了 -> Bを進める
		}
		else if (j >= N_B) {
			advanceA = true;  // B完了 -> Aを進める
		}
		else {
			// IDによるガイド (ここが最重要修正)
			// ストランドIDが有効(>=0)な場合、IDの一致を強力なヒントにする

			bool match_A_Next_with_B_Curr = (vA_next.strandID != -1) && (vA_next.strandID == vB_curr.strandID);
			bool match_A_Curr_with_B_Next = (vA_curr.strandID != -1) && (vA_curr.strandID == vB_next.strandID);

			if (match_A_Next_with_B_Curr) {
				// Aの次が、Bの現在と同じID -> Aが遅れているのでAを進める
				advanceA = true;
			}
			else if (match_A_Curr_with_B_Next) {
				// Aの現在が、Bの次と同じID -> Bが遅れているのでBを進める
				advanceA = false;
			}
			else {
				// IDで決まらない場合は、従来の「距離判定」を行う
				float dist1 = glm::length2(vA_next.pos - vB_curr.pos); // Aを進めた場合の対角線
				float dist2 = glm::length2(vA_curr.pos - vB_next.pos); // Bを進めた場合の対角線
				advanceA = (dist1 < dist2);
			}
		}

		//三角形の生成
		unsigned int baseIdx = (unsigned int)meshVertices.size();

		if (advanceA) {
			// Aを進める: (A_curr, A_next, B_curr)
			meshVertices.push_back(vA_curr.pos);
			meshVertices.push_back(vA_next.pos);
			meshVertices.push_back(vB_curr.pos);

			meshIndices.push_back(baseIdx);
			meshIndices.push_back(baseIdx + 1);
			meshIndices.push_back(baseIdx + 2);
			i++;
		}
		else {
			// Bを進める: (A_curr, B_next, B_curr) -> 順序注意: A_curr, B_next, B_curr (CCW)
			// 正しい巻き順: A_curr -> B_next -> B_curr
			meshVertices.push_back(vA_curr.pos);
			meshVertices.push_back(vB_next.pos);
			meshVertices.push_back(vB_curr.pos);

			meshIndices.push_back(baseIdx);
			meshIndices.push_back(baseIdx + 1);
			meshIndices.push_back(baseIdx + 2);
			j++;
		}
	}
}

// 閉じたループ(Loop) と 開いた鎖(Strip) を縫い合わせる関数
// Loop: ドーナツの穴 (閉じてる)
// Strip: 三日月の内側 (開いてる)
void TreeSystem::StitchOpenStrip(
	const std::vector<SliceVertex>& loopClosed,
	const std::vector<SliceVertex>& stripOpen,
	std::vector<glm::vec3>& meshVertices,
	std::vector<unsigned int>& meshIndices
) {
	if (loopClosed.empty() || stripOpen.empty()) return;

	std::vector<SliceVertex> workingStrip = stripOpen;

	int N_Loop = (int)loopClosed.size();
	int N_Strip = (int)workingStrip.size();

	// 開始点の位置合わせ
	int startLoop = FindBestMatchIndex(loopClosed, workingStrip[0].strandID, workingStrip[0].pos);
	if (startLoop == -1) startLoop = 0;

	
	// 方向の自動検知と補正
	// Strip[0]->[1] のベクトルと、Loop[start]->[start+1] のベクトルを比較する
	if (N_Strip > 1 && N_Loop > 2) {
		glm::vec3 vStrip = workingStrip[1].pos - workingStrip[0].pos;

		// Loopの次の点 (循環考慮)
		glm::vec3 pLoopNext = loopClosed[(startLoop + 1) % N_Loop].pos;
		glm::vec3 pLoopPrev = loopClosed[(startLoop - 1 + N_Loop) % N_Loop].pos;

		float distNext = glm::distance(workingStrip[1].pos, pLoopNext);
		float distPrev = glm::distance(workingStrip[1].pos, pLoopPrev);

		// 「次の点」よりも「前の点」の方が近ければ、向きが逆である
		if (distPrev < distNext) {
			//std::cout << "      [StitchOpenStrip] Direction mismatch detected. Reversing Strip." << std::endl;
			std::reverse(workingStrip.begin(), workingStrip.end());

			// 反転したので開始点も再検索
			startLoop = FindBestMatchIndex(loopClosed, workingStrip[0].strandID, workingStrip[0].pos);
			if (startLoop == -1) startLoop = 0;
		}
	}

	// ジッパー処理
	int i = 0;
	int j = 0;
	int maxSteps = N_Loop + N_Strip;
	int steps = 0;

	while (j < N_Strip - 1 && steps < maxSteps) {
		steps++;

		int i_curr = (startLoop + i) % N_Loop;
		int i_next = (startLoop + i + 1) % N_Loop;

		int j_curr = j;
		int j_next = j + 1;

		const auto& vL_curr = loopClosed[i_curr];
		const auto& vL_next = loopClosed[i_next];
		const auto& vS_curr = workingStrip[j_curr];
		const auto& vS_next = workingStrip[j_next];

		// 距離コスト
		float d1 = glm::length2(vL_next.pos - vS_curr.pos); // Loopを進める
		float d2 = glm::length2(vL_curr.pos - vS_next.pos); // Stripを進める

		// IDガイド
		bool matchNextL = (vL_next.strandID != -1 && vL_next.strandID == vS_curr.strandID);
		bool matchNextS = (vL_curr.strandID != -1 && vL_curr.strandID == vS_next.strandID);

		bool advanceLoop = false;
		if (matchNextL) advanceLoop = true;
		else if (matchNextS) advanceLoop = false;
		else advanceLoop = (d1 < d2);

		unsigned int baseIdx = (unsigned int)meshVertices.size();

		if (advanceLoop) {
			// Loop側を進める (Stripはステイ)
			// 三角形の巻き順 (CCW) を守る
			// Strip側が「穴の壁」になる場合、外から見て裏返らないように注意が必要
			// ここでは標準的な (A, B, C) で生成
			meshVertices.push_back(vL_curr.pos);
			meshVertices.push_back(vL_next.pos);
			meshVertices.push_back(vS_curr.pos);

			meshIndices.push_back(baseIdx);
			meshIndices.push_back(baseIdx + 1);
			meshIndices.push_back(baseIdx + 2);
			i++;
		}
		else {
			// Strip側を進める
			meshVertices.push_back(vL_curr.pos);
			meshVertices.push_back(vS_next.pos);
			meshVertices.push_back(vS_curr.pos); // 順序調整

			meshIndices.push_back(baseIdx);
			meshIndices.push_back(baseIdx + 1);
			meshIndices.push_back(baseIdx + 2);
			j++;
		}
	}
}

// 多角形の符号付き面積を計算する（正ならCCW、負ならCW）
float TreeSystem::CalculateLoopArea(
	const std::vector<SliceVertex>& loop, 
	const glm::vec3& u, const glm::vec3& v) 
{
	float area = 0.0f;
	if (loop.size() < 3) return 0.0f;

	// 重心を基準に投影
	glm::vec3 center = ComputeCentroid(loop);

	for (size_t i = 0; i < loop.size(); ++i) {
		glm::vec3 p1 = loop[i].pos - center;
		glm::vec3 p2 = loop[(i + 1) % loop.size()].pos - center;

		float x1 = glm::dot(p1, u);
		float y1 = glm::dot(p1, v);
		float x2 = glm::dot(p2, u);
		float y2 = glm::dot(p2, v);

		area += (x1 * y2 - x2 * y1);
	}
	return area * 0.5f;
}


void TreeSystem::AlignLoopRobust(
	const std::vector<SliceVertex>& target,
	std::vector<SliceVertex>& source,
	const glm::vec3& u_axis,
	const glm::vec3& v_axis
) {
	if (target.empty() || source.empty()) return;

	// 巻き方向の統一 
	float areaTarget = CalculateLoopArea(target, u_axis, v_axis);
	float areaSource = CalculateLoopArea(source, u_axis, v_axis);

	if (std::abs(areaTarget) > 1e-6 && std::abs(areaSource) > 1e-6) {
		if ((areaTarget > 0) != (areaSource > 0)) {
			std::reverse(source.begin(), source.end());
		}
	}

	// 開始位置の調整 (ID優先 & 距離ペナルティ)
	int bestOffset = -1;
	float bestScore = FLT_MAX; // 小さいほど良い

	// 重心距離を計算
	glm::vec3 tCenter = ComputeCentroid(target);

	for (int i = 0; i < source.size(); ++i) {
		float score = 0.0f;

		// 距離コスト: ターゲットの0番目との距離
		float distSq = glm::length2(source[i].pos - target[0].pos);
		score += distSq;

		// IDボーナス: IDが一致すればスコアを劇的に下げる
		if (source[i].strandID != -1 && source[i].strandID == target[0].strandID) {
			score -= 1000.0f;

			if (score < bestScore) {
				bestScore = score;
				bestOffset = i;
			}
		}
	}

	if (bestOffset > 0) {
		std::rotate(source.begin(), source.begin() + bestOffset, source.end());
	}
}

// 輪郭の重心を使って三角形の扇（Triangle Fan）を作り、蓋をする
void TreeSystem::GenerateCapMesh(
	const std::vector<SliceVertex>& loop,
	bool reverseWinding,
	std::vector<glm::vec3>& meshVertices,
	std::vector<unsigned int>& meshIndices
) {
	if (loop.size() < 3) {
		//警告ログ
		//std::cout << "[GenerateCapMesh] Skip: Loop too small (" << loop.size() << ")" << std::endl;
		return;
	}

	// 重心（Center）を計算
	glm::vec3 center(0.0f);
	for (const auto& v : loop) {
		center += v.pos;
	}
	center /= (float)loop.size();

	// 重心頂点を追加
	unsigned int centerIdx = (unsigned int)meshVertices.size();
	meshVertices.push_back(center);

	// ループ頂点を追加し、インデックスを記録
	std::vector<unsigned int> loopIndices;
	for (const auto& v : loop) {
		loopIndices.push_back((unsigned int)meshVertices.size());
		meshVertices.push_back(v.pos);
	}

	// 扇状（Triangle Fan）に面を張る
	int N = (int)loop.size();
	for (int i = 0; i < N; ++i) {
		unsigned int idxCurr = loopIndices[i];
		unsigned int idxNext = loopIndices[(i + 1) % N];

		if (reverseWinding) {
			// 底面用（下を向くように）: Center -> Next -> Curr
			meshIndices.push_back(centerIdx);
			meshIndices.push_back(idxNext);
			meshIndices.push_back(idxCurr);
		}
		else {
			// 上面用（上を向くように）: Center -> Curr -> Next
			meshIndices.push_back(centerIdx);
			meshIndices.push_back(idxCurr);
			meshIndices.push_back(idxNext);
		}
	}
}

// ループ（またはストリップ）の頂点数が minVertices 未満なら、補間して増やす
void TreeSystem::ResampleLoop(
	std::vector<SliceVertex>& loop,
	int minVertices,
	bool isClosed // 閉じたループとして扱うか
) {
	
	if (loop.empty()) return;

	//  頂点が1つしかない場合、コピー
	if (loop.size() == 1 && minVertices >= 2) {
		loop.push_back(loop[0]); // 同じ点を末尾に追加
		// ログを出して状況を知らせる
		//std::cout << "  [Resample] Single point detected. Duplicated to start interpolation." << std::endl;
	}

	if (loop.size() >= minVertices || loop.size() < 2) return;

	int currentSize = (int)loop.size();
	int needed = minVertices - currentSize;

	// 必要な数だけ、最も長いエッジの中点に頂点を挿入していく
	// 作業用バッファ
	std::vector<SliceVertex> newLoop = loop;

	for (int k = 0; k < needed; ++k) {
		float maxDistSq = -1.0f;
		int splitIndex = -1;

		// 一番長いエッジを探す
		int n = (int)newLoop.size();
		int loopEnd = isClosed ? n : n - 1;

		for (int i = 0; i < loopEnd; ++i) {
			int next = (i + 1) % n;
			float d = glm::length2(newLoop[next].pos - newLoop[i].pos);
			if (d > maxDistSq) {
				maxDistSq = d;
				splitIndex = i;
			}
		}

		if (splitIndex != -1) {
			int next = (splitIndex + 1) % n;
			const auto& v1 = newLoop[splitIndex];
			const auto& v2 = newLoop[next];

			// 中点生成
			SliceVertex mid;
			mid.pos = (v1.pos + v2.pos) * 0.5f;
			mid.strandID = v1.strandID; // IDは片方を継承（または-1）
			mid.isInnerArc = v1.isInnerArc; // 属性継承

			// 挿入
			newLoop.insert(newLoop.begin() + splitIndex + 1, mid);
		}
	}
	loop = newLoop;
}

void DetectSliceEvent(
	const SliceData& prev,
	SliceData& curr
) {
	if (prev.contours.size() == 1 && curr.contours.size() > 1) {
		curr.event = SliceEvent::BranchStart;
	}
	else if (prev.contours.size() > 1 && curr.contours.size() == 1) {
		curr.event = SliceEvent::MergeStart;
	}
	else {
		curr.event = SliceEvent::None;
	}
}


// 2つのループの開始位置を幾何学的に合わせる
void TreeSystem::AlignLoopsGeometric(
	const std::vector<SliceVertex>& target, // 基準（動かさない）
	std::vector<SliceVertex>& source        // 回転させる方
) {
	if (target.empty() || source.empty()) return;

	int nT = (int)target.size();
	int nS = (int)source.size();

	// 重心合わせ
	glm::vec3 cT = ComputeCentroid(target);
	glm::vec3 cS = ComputeCentroid(source);
	glm::vec3 diff = cT - cS;

	float minTotalDistSq = FLT_MAX;
	int bestOffset = 0;

	// sourceの開始位置を 0 ～ nS-1 までずらしてテスト
	for (int offset = 0; offset < nS; ++offset) {
		float currentDistSq = 0.0f;

		// サンプリング数（頂点数が多い場合は適当に間引く）
		int samples = std::min(nT, nS);

		for (int i = 0; i < samples; ++i) {
			// target[i] に対応する source のインデックス
			// 0.0~1.0 のパラメトリック位置で対応させる
			float t = (float)i / samples;

			int idxT = (int)(t * nT) % nT;
			int idxS = (offset + (int)(t * nS)) % nS;

			// 重心補正した位置で距離比較
			float d2 = glm::length2(target[idxT].pos - (source[idxS].pos + diff));
			currentDistSq += d2;
		}

		if (currentDistSq < minTotalDistSq) {
			minTotalDistSq = currentDistSq;
			bestOffset = offset;
		}
	}

	// ベストなオフセットで回転させる
	if (bestOffset > 0) {
		std::rotate(source.begin(), source.begin() + bestOffset, source.end());
		// std::cout << "  [AlignGeometric] Rotated source by " << bestOffset << std::endl;
	}
}

// 幾何学的距離に基づいて縫い合わせる (最短対角線法)
void TreeSystem::StitchGeometric(
	const std::vector<SliceVertex>& loopA, // Prev
	const std::vector<SliceVertex>& loopB, // Curr
	std::vector<glm::vec3>& meshVertices,
	std::vector<unsigned int>& meshIndices
) {
	if (loopA.empty() || loopB.empty()) return;

	std::vector<SliceVertex> alignedB = loopB;

	// 事前アライメント (LoopBを回転させてLoopAに合わせる)
	AlignLoopsGeometric(loopA, alignedB);

	// ジッパー処理 (Greedy Shortest Diagonal)
	int N_A = (int)loopA.size();
	int N_B = (int)alignedB.size();

	int i = 0;
	int j = 0;

	// AとBを一周するまで回す
	// 終了条件: 両方が一周し終わるまで
	while (i < N_A || j < N_B) {

		// 現在のインデックス (剰余演算でループ)
		int i_curr = i % N_A;
		int i_next = (i + 1) % N_A;
		int j_curr = j % N_B;
		int j_next = (j + 1) % N_B;

		const auto& vA_curr = loopA[i_curr];
		const auto& vA_next = loopA[i_next];
		const auto& vB_curr = alignedB[j_curr];
		const auto& vB_next = alignedB[j_next];

		bool advanceA = false;

		// どちらもまだ終わっていない場合 -> 距離比較
		if (i < N_A && j < N_B) {
			// 候補となる2つの対角線の長さ
			// Aを進める場合の対角線 (A_next -> B_curr)
			float d_advanceA = glm::length2(vA_next.pos - vB_curr.pos);

			// Bを進める場合の対角線 (A_curr -> B_next)
			float d_advanceB = glm::length2(vA_curr.pos - vB_next.pos);

			// 短い方を採用する (これによりねじれ/交差を防ぐ)
			if (d_advanceA < d_advanceB) {
				advanceA = true;
			}
			else {
				advanceA = false;
			}
		}
		// 片方が終わっている場合 -> 残っている方を進める
		else if (i < N_A) {
			advanceA = true;
		}
		else { // j < N_B
			advanceA = false;
		}

		// 三角形生成
		unsigned int baseIdx = (unsigned int)meshVertices.size();

		if (advanceA) {
			// Aを進める: (A_curr, A_next, B_curr)
			meshVertices.push_back(vA_curr.pos);
			meshVertices.push_back(vA_next.pos);
			meshVertices.push_back(vB_curr.pos);
			i++;
		}
		else {
			// Bを進める: (A_curr, B_next, B_curr) 
			meshVertices.push_back(vA_curr.pos);
			meshVertices.push_back(vB_next.pos);
			meshVertices.push_back(vB_curr.pos);
			j++;
		}

		meshIndices.push_back(baseIdx);
		meshIndices.push_back(baseIdx + 1);
		meshIndices.push_back(baseIdx + 2);
	}
}


void TreeSystem::StitchSlices(
	const SliceData& prev,
	const SliceData& curr,
	std::vector<glm::vec3>& meshVertices,
	std::vector<unsigned int>& meshIndices,
	int& currentIndex
) {
	// PrevかCurrが空なら何もしない
	if (prev.contours.empty() || curr.contours.empty()) return;

	// 島（Contour）同士のマッチング

	// Currの方が多い or 同じ (分岐 1->N, N->N)
	if (curr.contours.size() >= prev.contours.size()) {
		for (const auto& cIsland : curr.contours) {
			// 一番近い Prev島を探す
			const Contour* bestPrev = nullptr;
			float minDist = FLT_MAX;

			for (const auto& pIsland : prev.contours) {
				float d = glm::length2(cIsland.centroid - pIsland.centroid);
				if (d < minDist) {
					minDist = d;
					bestPrev = &pIsland;
				}
			}

			if (bestPrev) {
				StitchIslands(
					*bestPrev,
					cIsland,
					prev,
					curr,
					meshVertices, 
					meshIndices, 
					currentIndex);
			}
		}
	}
	// Prevの方が多い (合流 N->1)
	else {
		for (const auto& pIsland : prev.contours) {
			// 一番近い Curr島を探す
			const Contour* bestCurr = nullptr;
			float minDist = FLT_MAX;

			for (const auto& cIsland : curr.contours) {
				float d = glm::length2(pIsland.centroid - cIsland.centroid);
				if (d < minDist) {
					minDist = d;
					bestCurr = &cIsland;
				}
			}

			if (bestCurr) {
				StitchIslands(
					pIsland, 
					*bestCurr, 
					prev,
					curr,
					meshVertices, 
					meshIndices, 
					currentIndex);
			}
		}
	}


}

// 1つのループを、フラグ(isInnerArc)に基づいて「外側用」と「内側用」に分離する
// needReverse: Holeとして使う場合、巻き方向を反転（CCW -> CW）させる必要があるため
void SplitContourByFlag(
	const std::vector<SliceVertex>& sourceLoop,
	std::vector<SliceVertex>& outOuterPart,
	std::vector<SliceVertex>& outInnerPart,
	bool needReverseForHole
) {
	outOuterPart.clear();
	outInnerPart.clear();

	if (sourceLoop.empty()) return;

	// 単純な分離
	for (const auto& v : sourceLoop) {
		if (v.isInnerArc) {
			outInnerPart.push_back(v);
		}
		else {
			outOuterPart.push_back(v);
		}
	}

	if (needReverseForHole && !outInnerPart.empty()) {
		std::reverse(outInnerPart.begin(), outInnerPart.end());
	}
}


// C型の輪郭を「isInnerArc」フラグに基づいて、外側(Outer)と内側(Inner)のストリップに分割する
bool TreeSystem::SplitContourGeometrically(
	const std::vector<SliceVertex>& loop,
	std::vector<SliceVertex>& outOuterStrip,
	std::vector<SliceVertex>& outInnerStrip
) {
	if (loop.size() < 3) return false;
	outOuterStrip.clear();
	outInnerStrip.clear();
	int n = (int)loop.size();

	// フラグを見て、Outer(false) と Inner(true) の境界を探す
	int rotationOffset = -1;

	// 「前がInnerで、今がOuter」になる場所（Outerの開始点）を探す
	for (int i = 0; i < n; ++i) {
		int prev = (i - 1 + n) % n;
		if (loop[prev].isInnerArc && !loop[i].isInnerArc) {
			rotationOffset = i;
			break;
		}
	}

	// もし境界が見つからない（全部Inner、または全部Outer）場合は分割不能
	if (rotationOffset == -1) {
		return false;
	}

	// 回転させた一時リストを作成 (Outerから始まるようにする)
	std::vector<SliceVertex> rotatedLoop;
	for (int i = 0; i < n; ++i) {
		rotatedLoop.push_back(loop[(rotationOffset + i) % n]);
	}

	// 順に振り分け
	bool fillingOuter = true;
	for (const auto& v : rotatedLoop) {
		if (fillingOuter) {
			if (!v.isInnerArc) {
				// 外側フラグなら OuterStrip へ
				outOuterStrip.push_back(v);
			}
			else {
				// Outer -> Inner の切り替わり点
				// 境界の頂点は「外」と「内」の両方で共有する（隙間防止）
				if (!outOuterStrip.empty()) outInnerStrip.push_back(outOuterStrip.back());

				outInnerStrip.push_back(v);
				fillingOuter = false; // モード切替
			}
		}
		else {
			if (v.isInnerArc) {
				// 内側フラグなら InnerStrip へ
				outInnerStrip.push_back(v);
			}
			else {
				// Inner -> Outer（終端）
			}
		}
	}

	// InnerStripは、ドーナツのHole(CW)と接続するため、逆順(CCW->CW)にしておく
	std::reverse(outInnerStrip.begin(), outInnerStrip.end());

	return true;
}

// C型（1ループ）を、仮想的なドーナツ型（2ループ）に変換する
// isInnerArcの区間を「Hole」、それ以外を「Outer」とする
bool TreeSystem::ConvertCToVirtualDonut(
	const std::vector<SliceVertex>& cLoop,
	std::vector<SliceVertex>& outVirtualOuter,
	std::vector<SliceVertex>& outVirtualHole
) {
	if (cLoop.size() < 3) return false;
	outVirtualOuter.clear();
	outVirtualHole.clear();

	int n = (int)cLoop.size();

	// Inner区間の検出
	int idxStartInner = -1;
	int idxEndInner = -1;

	for (int i = 0; i < n; ++i) {
		int next = (i + 1) % n;
		bool currIsInner = cLoop[i].isInnerArc;
		bool nextIsInner = cLoop[next].isInnerArc;

		if (!currIsInner && nextIsInner) idxStartInner = next;
		else if (currIsInner && !nextIsInner) idxEndInner = i;
	}

	if (idxStartInner == -1 || idxEndInner == -1) return false;

	// Outerループの構築
	{
		int curr = idxEndInner;
		bool isFirst = true;
		while (true) {
			outVirtualOuter.push_back(cLoop[curr]);
			if (!isFirst && curr == idxStartInner) break;
			isFirst = false;
			curr = (curr + 1) % n;
		}
	}

	// Holeループの構築 (純粋なInner区間のみ抽出)
	{
		int curr = idxStartInner;
		while (true) {
			outVirtualHole.push_back(cLoop[curr]);
			if (curr == idxEndInner) break;
			curr = (curr + 1) % n;
		}

		// リサンプリング実行
		ResampleLoop(outVirtualHole, 12, false);

		// 穴はCWにするため反転
		std::reverse(outVirtualHole.begin(), outVirtualHole.end());
	}

	ResampleLoop(outVirtualOuter, 16, true); 

	return true;
}


// ヘルパー: 島と島の中身（外周・穴）を縫い合わせる関数
void TreeSystem::StitchIslands(
	const Contour& prev,
	const Contour& curr,
	const SliceData prevSlice,
	const SliceData currSlice,
	std::vector<glm::vec3>& verts,
	std::vector<unsigned int>& inds,
	int& idx
) {
	size_t numPH = prev.holes.size();
	size_t numCH = curr.holes.size();

	// 穴の数が同じ (通常接続)
	if (numPH == numCH) {

		// Outer接続の直前
		int bestOffsetOuter = -1;

		if (!prev.outer.empty() && !curr.outer.empty()) {
			// 距離が一番近いペアを探す
			float minD = FLT_MAX;
			int matchIdx = -1;
			for (int i = 0; i < curr.outer.size(); ++i) {
				float d = glm::distance(prev.outer[0].pos, curr.outer[i].pos);
				if (d < minD) { minD = d; matchIdx = i; }
			}
		}

		// 外周
		StitchGeometric(prev.outer, curr.outer, verts, inds);

		// 穴同士 (距離でマッチング)
		std::vector<bool> usedC(numCH, false);
		for (size_t p = 0; p < numPH; ++p) {
			int bestC = -1;
			float minDist = FLT_MAX;
			glm::vec3 pCenter = ComputeCentroid(prev.holes[p]);

			for (size_t c = 0; c < numCH; ++c) {
				if (usedC[c]) continue;
				float d = glm::distance(pCenter, ComputeCentroid(curr.holes[c]));
				if (d < minDist) { minDist = d; bestC = (int)c; }
			}

			if (bestC != -1) {
				StitchGeometric(prev.holes[p], curr.holes[bestC], verts, inds);
				usedC[bestC] = true;
			}
		}
		return; // 終了
	}

	//C型 -> ドーナツ型 (Prev=C, Curr=Donut)
	if (numPH == 0 && numCH == 1) {
		
		std::vector<SliceVertex> prevVirtualOuter;
		std::vector<SliceVertex> prevVirtualHole;

		// Prev(C型) を仮想的なドーナツに変換
		bool success = ConvertCToVirtualDonut(prev.outer, prevVirtualOuter, prevVirtualHole);


		// 軸情報を渡して、回転方向を確実に合わせる
		// // Prevの軸を使う（Currでも良いが、同じ平面上で比較するため）
		glm::vec3 u = prevSlice.u_axis; // または引数で受け取る
		glm::vec3 v = prevSlice.v_axis;

		AlignLoopRobust(curr.holes[0], prevVirtualHole, u, v);
		AlignLoopRobust(curr.outer, prevVirtualOuter, u, v);

		// Outer接続の直前
		int bestOffsetOuter = -1;
		
		if (!prev.outer.empty() && !curr.outer.empty()) {
			// 距離が一番近いペアを探す
			float minD = FLT_MAX;
			int matchIdx = -1;
			for (int i = 0; i < curr.outer.size(); ++i) {
				float d = glm::distance(prev.outer[0].pos, curr.outer[i].pos);
				if (d < minD) { minD = d; matchIdx = i; }
			}
		}

		if (success) {
			AlignLoopRobust(curr.holes[0], prevVirtualHole, u, v);
			AlignLoopRobust(curr.outer, prevVirtualOuter, u, v); 

			// (A) 外周
			StitchZipper(prevVirtualOuter, curr.outer, verts, inds, idx);
			// (B) 穴
			StitchZipper(prevVirtualHole, curr.holes[0], verts, inds, idx);
		}
		else {
			StitchZipper(prev.outer, curr.outer, verts, inds, idx);
		}
		return;
	}

	
	//ドーナツ型 -> C型 (Prev=Donut, Curr=C)
	if (numPH == 1 && numCH == 0) {
		//std::cout << "    [TRANSITION] Donut -> C-Shape (Virtualizing Curr)" << std::endl;

		std::vector<SliceVertex> currVirtualOuter;
		std::vector<SliceVertex> currVirtualHole;

		// Curr(C型) を仮想的なドーナツに変換
		bool success = ConvertCToVirtualDonut(curr.outer, currVirtualOuter, currVirtualHole);

		// Outer接続の直前
		int bestOffsetOuter = -1;

		if (!prev.outer.empty() && !curr.outer.empty()) {
			// 距離が一番近いペアを探す
			float minD = FLT_MAX;
			int matchIdx = -1;
			for (int i = 0; i < curr.outer.size(); ++i) {
				float d = glm::distance(prev.outer[0].pos, curr.outer[i].pos);
				if (d < minD) { minD = d; matchIdx = i; }
			}
		}

		glm::vec3 u = prevSlice.u_axis; // または引数で受け取る
		glm::vec3 v = prevSlice.v_axis;

		if (success) {
			AlignLoopRobust(prev.holes[0], currVirtualHole, u, v);
			AlignLoopRobust(prev.outer, currVirtualOuter, u, v);

			// (A) 外周
			StitchZipper(prev.outer, currVirtualOuter, verts, inds, idx);
			// (B) 穴
			StitchZipper(prev.holes[0], currVirtualHole, verts, inds, idx);
		}
		else {
			StitchZipper(prev.outer, curr.outer, verts, inds, idx);
		}
		return;
	}
	
	//その他 (分岐・合流が複雑な場合)
	// フォールバック: とりあえずOuterだけつないでおく
	StitchZipper(prev.outer, curr.outer, verts, inds, idx);
}


//メッシュ生成のメインループ
void TreeSystem::generateSegmentMesh(
	TreeNode* parent,
	TreeNode* child,
	std::vector<glm::vec3>& meshVertices,
	std::vector<unsigned int>& meshIndices,
	int& currentIndex,
	bool obstaclesEnabled
) {

	// 計測用変数 (staticにして関数呼び出し間で保持)
	static double timeInterp = 0.0; // 補間時間
	static double timeAlpha = 0.0; // AlphaShape時間
	static int callCount = 0;       // 呼び出し回数カウンタ


	// ステップ数の動的決定
	float dist = glm::distance(parent->position, child->position);
	int steps = static_cast<int>(dist / STEP_LENGTH);
	if (steps < 1) steps = 1;
	if (steps > 20) steps = 20;

	// 前のステップの状態を保存する変数
	SliceData prevSlice;

	bool usedCache = false;
	// 親が有効で、かつ親のスライスデータ（Contours）が存在する場合
	if (parent && !parent->sliceGeometry.contours.empty()) {
		// キャッシュを利用 
		prevSlice = parent->sliceGeometry;
		usedCache = true;
	}

	//初回(t=0)の計算
	// 親ノードにすでに計算済みのスライス情報があれば、それを流用する
	{

		if (!usedCache) {

			// --- 計測開始 (Interpolation) --- //
			auto t_start_interp = std::chrono::high_resolution_clock::now();

			std::vector<SliceVertex> strands0;
			glm::vec3 c, u, v;

			// 補間
			InterpolateStrands(parent, child, 0.0f, strands0, c, u, v);

			//軸の安全性チェック (NaN / ゼロベクトル対策)
			bool isAxisInvalid = false;
			if (std::isnan(u.x) || glm::length2(u) < 0.0001f) isAxisInvalid = true;
			if (std::isnan(v.x) || glm::length2(v) < 0.0001f) isAxisInvalid = true;
			if (std::isnan(c.x)) isAxisInvalid = true;

			// --- 計測終了 (Interpolation) & 開始 (AlphaShape) --- //
			auto t_end_interp = std::chrono::high_resolution_clock::now();


			// AlphaShape
			getAlphaShapeContours(
				strands0,
				DELAUNAY_THRESHOLD,
				(obstaclesEnabled ? m_pObstacleObj : nullptr),
				c, u, v,
				prevSlice.contours,
				obstaclesEnabled);

			// AlphaShape (緩和) - 失敗時リトライ
			if (prevSlice.contours.empty() && !strands0.empty()) {
				getAlphaShapeContours(
					strands0,
					LOOSE_DELAUNAY_THRESHOLD,
					(obstaclesEnabled ? m_pObstacleObj : nullptr),
					c, u, v,
					prevSlice.contours,
					obstaclesEnabled);
			}

			// --- 計測終了 (AlphaShape) ---
			auto t_end_alpha = std::chrono::high_resolution_clock::now();

			// 加算
			timeInterp += std::chrono::duration<double, std::milli>(t_end_interp - t_start_interp).count();
			timeAlpha += std::chrono::duration<double, std::milli>(t_end_alpha - t_end_interp).count();

			prevSlice.centerPos = c;
			prevSlice.u_axis = u;
			prevSlice.v_axis = v;

			// デバッグ描画リストへの追加
			for (size_t k = 0; k < prevSlice.contours.size(); ++k) {
				const auto& c0 = prevSlice.contours[k];
				m_DebugSliceContours.push_back(c0.outer);
				for (const auto& h : c0.holes) m_DebugSliceContours.push_back(h);

			}
		}

		// デバッグ用ラインへの追加 (キャッシュを使った場合も描画リストには入れる)
		if (usedCache) {
			for (const auto& contour : prevSlice.contours) {
				m_DebugSliceContours.push_back(contour.outer);
				for (const auto& h : contour.holes) m_DebugSliceContours.push_back(h);
			}
		}
	}

	//補間ループ
	for (int i = 1; i <= steps; ++i) {
		float t = (float)i / (float)steps;

		// --- 計測開始 (Interpolation) --- //
		auto t_start_interp = std::chrono::high_resolution_clock::now();

		// 補間
		std::vector<SliceVertex> currStrands;
		glm::vec3 center, u_axis, v_axis;

		InterpolateStrands(
			parent,
			child,
			t,
			currStrands,
			center,
			u_axis,
			v_axis
		);

		// --- 計測終了 (Interpolation) & 開始 (AlphaShape) --- //
		auto t_end_interp = std::chrono::high_resolution_clock::now();

		// 輪郭抽出
		SliceData currSlice;

		getAlphaShapeContours(
			currStrands,
			DELAUNAY_THRESHOLD,
			(obstaclesEnabled ? m_pObstacleObj : nullptr),
			center,
			u_axis,
			v_axis,
			currSlice.contours,
			obstaclesEnabled
		);

		// もし輪郭が生成されず、かつストランド粒子は存在する場合
		if (currSlice.contours.empty() && !currStrands.empty()) {
			//std::cout << "  [Retry] Using LOOSE threshold at t=" << t << std::endl; // デバッグ用

			getAlphaShapeContours(
				currStrands,
				LOOSE_DELAUNAY_THRESHOLD, // 緩い閾値を使用
				(obstaclesEnabled ? m_pObstacleObj : nullptr),
				center,
				u_axis,
				v_axis,
				currSlice.contours,
				obstaclesEnabled
			);
		}

		// --- 計測終了 (AlphaShape) --- //
		auto t_end_alpha = std::chrono::high_resolution_clock::now();

		// 加算
		timeInterp += std::chrono::duration<double, std::milli>(t_end_interp - t_start_interp).count();
		timeAlpha += std::chrono::duration<double, std::milli>(t_end_alpha - t_end_interp).count();

		currSlice.centerPos = center; 
		currSlice.u_axis = u_axis;
		currSlice.v_axis = v_axis;


		DetectSliceEvent(prevSlice, currSlice);
	
		// メッシュ縫合
		//ここでprevSliceとcurrSliceをつなぐ
		StitchSlices(
			prevSlice,
			currSlice,
			meshVertices,
			meshIndices,
			currentIndex);

		//次へ
		prevSlice = currSlice;
	}

	child->sliceGeometry = prevSlice;

	//先端の蓋
	if (child->children.empty()) {
		for (const auto& contour : prevSlice.contours) {
			//GenerateCapMesh(contour.outer, false, meshVertices, meshIndices);
			for (const auto& h : contour.holes) {
				//GenerateCapMesh(h, false, meshVertices, meshIndices);
			}
		}
	}


	// 定期出力
	callCount++;
	if (callCount % 100 == 0) {
		std::cout << "[Profile Detail] Interp: " << timeInterp << "ms | AlphaShape: " << timeAlpha << "ms" << std::endl;
		// 必要ならリセットしないことで累積を見る、あるいはここでリセットして区間計測にする
		// timeInterp = 0; timeAlpha = 0;
		}
}


void TreeSystem::exportToOBJ(const std::string& filename) {
	
	std::vector<MeshData> meshes;
	{
		MeshData treeMesh;
		treeMesh.name = "Tree_Mesh";
		treeMesh.vertices = m_MeshVertices;
		treeMesh.indices = m_MeshIndices;
		meshes.push_back(treeMesh);
	}
	// 障害物のメッシュを追加 
	if (m_pObstacleObj) {
		// Obstacle* を MeshObstacle* にキャスト
		MeshObstacle* meshObs = static_cast<MeshObstacle*>(m_pObstacleObj);

		MeshData obsMesh;
		obsMesh.name = "Obstacle_Mesh";

		// 障害物の頂点データを取得
		std::vector<glm::vec3> obsVerts;
		meshObs->getTriangleVertices(obsVerts);

		obsMesh.vertices = obsVerts;

		// インデックスの生成
		for (unsigned int i = 0; i < obsVerts.size(); ++i) {
			obsMesh.indices.push_back(i);
		}

		meshes.push_back(obsMesh);
	}

	OBJExporter::saveMultiple(filename, meshes);
}

// --- 描画関数群 --- //
void TreeSystem::drawEdges() {
	glBindVertexArray(m_VAO_Edges);
	glDrawArrays(GL_LINES, 0, m_EdgeVertexCount);
	glBindVertexArray(0);
}

void TreeSystem::drawNodes() {
	if (m_VAO_Nodes == 0) return;

	glBindVertexArray(m_VAO_Nodes);
	glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Nodes); 

	// 通常ノード (青色 / 小さい)
	// データを転送
	glBufferData(GL_ARRAY_BUFFER,
		m_DebugNormalNodes.size() * sizeof(glm::vec3),
		m_DebugNormalNodes.data(),
		GL_DYNAMIC_DRAW);
	// 通常の色
	glPointSize(8.0f);
	glDrawArrays(GL_POINTS, 0, (GLsizei)m_DebugNormalNodes.size());
	glBindVertexArray(0);
}

void TreeSystem::drawSubNodes() {
	if (m_VAO_Nodes == 0) return;

	glBindVertexArray(m_VAO_Nodes);
	glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Nodes); 

	if (!m_DebugGeneratedNodes.empty()) {
		// 深度テストを無効化
		GLboolean depthEnabled;
		glGetBooleanv(GL_DEPTH_TEST, &depthEnabled);
		glDisable(GL_DEPTH_TEST);

		// データを生成ノードのものに入れ替える
		glBufferData(GL_ARRAY_BUFFER,
			m_DebugGeneratedNodes.size() * sizeof(glm::vec3),
			m_DebugGeneratedNodes.data(),
			GL_DYNAMIC_DRAW);

		glPointSize(8.0f);
		glDrawArrays(GL_POINTS, 0, (GLsizei)m_DebugGeneratedNodes.size());
		glPointSize(1.0f);
	}

	glBindVertexArray(0);
}
void TreeSystem::drawStrands() {
	glBindVertexArray(m_VAO_Strands);
	glPointSize(8.0f);
	glDrawArrays(GL_POINTS, 0, m_TotalStrandCount);
	glBindVertexArray(0);
	glPointSize(1.0f);
}
void TreeSystem::drawProfile() {
	glBindVertexArray(m_VAO_Profile);
	for (int i = 0; i < m_TotalProfileVertexCount / m_ProfileVertexCount; ++i) {
		glDrawArrays(GL_LINE_LOOP, i * m_ProfileVertexCount, m_ProfileVertexCount);
	}
	glBindVertexArray(0);
}
void TreeSystem::drawStrandCurves() {
	glBindVertexArray(m_VAO_StrandCurves);
	glDrawArrays(GL_LINES, 0, m_StrandCurveVertexCount);
	glBindVertexArray(0);
}
void TreeSystem::drawSliceBoundaries() {
	glBindVertexArray(m_VAO_Slices);
	int offset = 0;
	for (int count : m_SliceCounts) {
		if (count > 0) {
			glDrawArrays(GL_LINE_LOOP, offset, count);
			offset += count;
		}
	}
	glBindVertexArray(0);
}

void TreeSystem::drawMesh() {
	if (m_MeshIndexCount > 0) {
		glBindVertexArray(m_VAO_Mesh);
		glDrawElements(GL_TRIANGLES, m_MeshIndexCount, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);
	}
}

void TreeSystem::drawObstacle() {
	if (m_VAO_Obstacle == 0) return;

	glBindVertexArray(m_VAO_Obstacle);
	// ワイヤーフレームで描画
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDrawArrays(GL_TRIANGLES, 0, m_ObstacleVertexCount);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBindVertexArray(0);
}

void TreeSystem::drawDebugTriangles() {
	if (m_DebugTriangleVertexCount == 0) return;
	glBindVertexArray(m_VAO_DebugTriangles);
	glDisable(GL_DEPTH_TEST);
	glLineWidth(3.0f);
	// ワイヤーフレームで描画
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDrawArrays(GL_TRIANGLES, 0, m_DebugTriangleVertexCount);
	// 設定を元に戻す
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glLineWidth(1.0f);
	glEnable(GL_DEPTH_TEST); // 深度テストを戻す
	glBindVertexArray(0);
}

void TreeSystem::drawDebugSlices() {
	if (m_SliceCounts.empty() || m_VAO_Slices == 0) return;
	glBindVertexArray(m_VAO_Slices);
	// 最前面に描画するための設定
	GLboolean depthEnabled;
	glGetBooleanv(GL_DEPTH_TEST, &depthEnabled);
	glDisable(GL_DEPTH_TEST); 

	glLineWidth(3.0f); 
	int offset = 0;
	for (int count : m_SliceCounts) {
		if (count > 0) {
			glDrawArrays(GL_LINE_LOOP, offset, count);
			offset += count;
		}
	}

	
	glPointSize(6.0f); // 点を大きく
	offset = 0; // オフセットをリセット
	for (int count : m_SliceCounts) {
		if (count > 0) {
			glDrawArrays(GL_POINTS, offset, count);
			offset += count;
		}
	}
	glPointSize(1.0f);
	glLineWidth(1.0f);
	if (depthEnabled) glEnable(GL_DEPTH_TEST);

	glBindVertexArray(0);
}