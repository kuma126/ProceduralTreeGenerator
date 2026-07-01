#pragma once

#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include <string>
#include "Geometry.h"
#include "ParticleSystem.h"
#include "Obstacle.h"
#include "MeshObstacle.h"

//頂点に付加する情報
struct VertexInfo {
    int strandID;
    bool isInnerCandidate = false;
};

//スライス上の頂点構造体
struct SliceVertex {
    glm::vec3 pos;
    int strandID;
    bool isInnerArc = false; // 障害物によって削られた境界かどうか
};


//輪郭構造体
struct Contour {
    int persistentID = -1;
    //輪郭を構成する頂点リスト(順序付き)
    std::vector<SliceVertex> vertices;

    // 外周（必ず1つ、CCW）(反時計回り)
    std::vector<SliceVertex> outer;
    // 穴（0個以上、CW）(時計回り)
    std::vector<std::vector<SliceVertex>> holes;

    //outerの重心
    glm::vec3 centroid; 
    float area = 0.0f;      // outer の符号付き面

    bool isOpen = false;
    bool hasHoles() const { return !holes.empty(); }
  
    //コンストラクタ
    Contour() : centroid(0.0f), area(0.0f){}
};

enum class SliceEvent {
    None,
    BranchStart,   // 1 → N が起きた
    MergeStart    // N → 1 が起きた
};

struct SliceData {
    //子の断面に含まれるすべての輪郭(幹一本なら1つ、分岐中なら2つ)
    std::vector<Contour> contours;

    //どの高さ(t)あるいはノードに属するか
    int parentNodeID;

    //断面の基準座標系(メッシュ生成時の法線計算に便利)
    glm::vec3 centerPos;
    glm::vec3 u_axis;
    glm::vec3 v_axis;

    SliceEvent event = SliceEvent::None;
};



class ParticleSystem;

// ノード構造体
struct TreeNode {
    enum Type { ROOT, JUNCTION, TIP };

    int id;
    Type type;
    glm::vec3 position;

    TreeNode* parent = nullptr;
    std::vector<TreeNode*> children;

    // このノードにおけるストランド粒子 (PBD適用後)
    std::vector<Point3D> strandParticles;

    //このノードで計算された輪郭情報
    //getAlphaShapeContoursの結果をここに格納
    SliceData sliceGeometry;

    // 動的半径
    float bundleRadius = 0.0f; // ストランド束の広がり
    float profileRadius = 0.0f; // プロファイル円の半径

    glm::vec3 u_axis = glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 v_axis = glm::vec3(0.0f, 0.0f, 1.0f);

    //自動生成されたノードかどうかを識別するフラグ
    bool isGenerated = false;

    //PBDの影響を受けない、参照用のストランド座標
    std::vector<Point3D> refStrandParticles;

    //コンストラクタ
    TreeNode(int _id, Type _type, glm::vec3 _pos, float _radius = 0.1f)
        : id(_id), type(_type), position(_pos) {

        // 初期化時に半径をセット
        this->profileRadius = _radius;
    }

    ~TreeNode() {
        for (auto c : children) delete c;
    }
};


// CSVの行データを一時的に保持する構造体
struct RawNodeData {
    int id;
    int parentId;
    glm::vec3 pos;
    float thickness;
};

class TreeSystem {
public:
    TreeSystem();
    ~TreeSystem();

    void init();

    // updateはシミュレーションの進行とVBO更新のみを行う
    void update(float profileRadius, int particlesPerTip, bool ObstaclesEnabled);

    bool loadSkeletonFromCSV(const std::string& filepath);

    // シミュレーション制御
    void startSimulation() { m_IsSimulating = true; }
    void stopSimulation() { m_IsSimulating = false; }
    bool isSimulating() const { return m_IsSimulating; }

    // 描画関数
    void drawEdges();
    void drawNodes();
    void drawSubNodes();
    void drawStrands();
    void drawProfile();
    void drawStrandCurves();
    void drawSliceBoundaries();
    void drawMesh();
    void drawObstacle();
    void drawDebugTriangles();//ドロネー三角形デバッグ用
    void drawDebugSlices();

    // OBJエクスポート
    void exportToOBJ(const std::string& filename);

private:

    TreeNode* CheckAndSubdivide(
        TreeNode* parent,
        TreeNode* child,
        Obstacle* pObstacle
    );

    void RenumberNodesRecursive(
        TreeNode* node,
        int& currentID
    );

    void RenumberTree();

    void buildTestTree();

    void RefineTreeRecursive(
        TreeNode* node,
        Obstacle* pObstacle,
        int depth
    );

    void PruneGeneratedNodes(TreeNode* node);

    void processNodeRecursive(
        TreeNode* node,
        float baseProfileRadius,
        int particlesPerTip,
        bool ObstaclesEnabled
    );

    // データ収集 (再帰)
    void collectParticlesRecursive(
        TreeNode* node, 
        std::vector<Point3D>& outParticles
    );
    void collectEdgesRecursive(
        TreeNode* node, 
        std::vector<glm::vec3>& outVertices
    );
    void collectProfilesRecursive(
        TreeNode* node, 
        std::vector<glm::vec3>& outVertices
    );
    void collectNodesForDebug(
        TreeNode* node
    );
    void collectTipsRecursive(
        TreeNode* node, 
        std::vector<TreeNode*>& outTips
    );

    void generateMeshRecursive(
        TreeNode* node,
        std::vector<glm::vec3>& meshVertices,
        std::vector<unsigned int>& meshIndices,
        int& currentIndex,
        bool obstaclesEnabled
    );
    void generateStrandCurves(
        std::vector<glm::vec3>& outCurveVertices
    );
    void getAlphaShapeContours(
        const std::vector<SliceVertex>& points,
        float threshold,
        Obstacle* pObstacle,
        const glm::vec3& centerPos,
        const glm::vec3& u_axis,
        const glm::vec3& v_axis,
        std::vector<Contour>& outContours,
        bool obstaclesEnabled
    );

    void InterpolateStrands(
        TreeNode* parent,
        TreeNode* child,
        float t,
        std::vector<SliceVertex>& outStrands,
        glm::vec3& outCenter,
        glm::vec3& outU,
        glm::vec3& outV
    );

private:

    int m_NextContourID = 0;

    int FindBestMatchIndex(
        const std::vector<SliceVertex>& loop, 
        int targetID, const glm::vec3& targetPos
    );

    void StitchZipper(
        const std::vector<SliceVertex>& loopA, 
        const std::vector<SliceVertex>& loopB, 
        std::vector<glm::vec3>& verts, 
        std::vector<unsigned int>& inds, int& idx
    );

    float CalculateLoopArea(
        const std::vector<SliceVertex>& loop,
        const glm::vec3& u, const glm::vec3& v
    );

    void ResampleLoop(
        std::vector<SliceVertex>& loop,
        int minVertices,
        bool isClosed // 閉じたループとして扱うか
    );


    void AlignLoopRobust(
        const std::vector<SliceVertex>& target,
        std::vector<SliceVertex>& source,
        const glm::vec3& u_axis, // 軸情報を追加で受け取る
        const glm::vec3& v_axis
    );


    bool ConvertCToVirtualDonut(
        const std::vector<SliceVertex>& cLoop,
        std::vector<SliceVertex>& outVirtualOuter,
        std::vector<SliceVertex>& outVirtualHole
    );

    float ComputeSignedArea(
        const std::vector<SliceVertex>& loop,
        const glm::vec3& centerPos,
        const glm::vec3& u_axis,
        const glm::vec3& v_axis
    );

    bool SplitContourGeometrically(
        const std::vector<SliceVertex>& loop,
        std::vector<SliceVertex>& outOuterStrip,
        std::vector<SliceVertex>& outInnerStrip
    );

    void StitchOpenStrip(
        const std::vector<SliceVertex>& loopClosed,
        const std::vector<SliceVertex>& stripOpen,
        std::vector<glm::vec3>& meshVertices,
        std::vector<unsigned int>& meshIndices
    );

    void AlignLoopsGeometric(
        const std::vector<SliceVertex>& target, // 基準（動かさない）
        std::vector<SliceVertex>& source        // 回転させる方
    );

    void StitchGeometric(
        const std::vector<SliceVertex>& loopA, // Prev
        const std::vector<SliceVertex>& loopB, // Curr
        std::vector<glm::vec3>& meshVertices,
        std::vector<unsigned int>& meshIndices
    );

    void StitchSlices(
        const SliceData& prev,
        const SliceData& curr,
        std::vector<glm::vec3>& verts,
        std::vector<unsigned int>& indices,
        int& idx
    );

    void StitchIslands(
        const Contour& prev,
        const Contour& curr,
        const SliceData prevSlice,
        const SliceData currSlice,
        std::vector<glm::vec3>& verts,
        std::vector<unsigned int>& inds,
        int& idx
    );

    void GenerateCapMesh(
        const std::vector<SliceVertex>& loop,
        bool reverseWinding,
        std::vector<glm::vec3>& meshVertices,
        std::vector<unsigned int>& meshIndices
    );

    void generateSegmentMesh(
        TreeNode* parent,
        TreeNode* child,
        std::vector<glm::vec3>& verts,
        std::vector<unsigned int>& indices,
        int& idx,
        bool obsEnabled
    );

    std::vector<std::vector<SliceVertex>> m_DebugSliceContours;

    int m_NextStrandID;


    // 乱数
    float getRandomFloat(float min, float max);

    // メンバ変数
    TreeNode* m_RootNode = nullptr;
    ParticleSystem* m_pParticleSystem;
    std::vector<PBDParticle> m_InitialParticlesData;

    // 状態
    bool m_IsSimulating = false;
    float m_CurrentProfileRadius;
    int m_CurrentParticlesPerTip;

    // 描画用バッファ
    std::vector<Point3D> m_AllStrandParticles;
    std::vector<glm::vec3> m_MeshVertices;
    std::vector<unsigned int> m_MeshIndices;

    GLuint m_VAO_Edges, m_VBO_Edges;
    int m_EdgeVertexCount;
    GLuint m_VAO_Nodes, m_VBO_Nodes;
    int m_NodeVertexCount;
    GLuint m_VAO_Strands, m_VBO_Strands;
    int m_TotalStrandCount;
    GLuint m_VAO_Profile, m_VBO_Profile;
    int m_ProfileVertexCount;
    int m_TotalProfileVertexCount;
    GLuint m_VAO_StrandCurves, m_VBO_StrandCurves;
    int m_StrandCurveVertexCount;
    GLuint m_VAO_Slices, m_VBO_Slices;
    std::vector<int> m_SliceCounts;
    int m_SliceTotalVertices;
    GLuint m_VAO_Mesh, m_VBO_Mesh, m_EBO_Mesh;
    int m_MeshIndexCount;

    // デバッグ描画用 (ドロネー三角形)
    unsigned int m_VAO_DebugTriangles = 0;
    unsigned int m_VBO_DebugTriangles = 0;
    int m_DebugTriangleVertexCount = 0;
    std::vector<glm::vec3> m_DebugTriangleVertices;

    // 障害物
    GLuint m_VAO_Obstacle, m_VBO_Obstacle;
    int m_ObstacleVertexCount;
    glm::vec3 m_ObstaclePos;
    float m_ObstacleRadius;

    Obstacle* m_pObstacleObj = nullptr;

    GLuint m_VAO_ProjectedStrands, m_VBO_ProjectedStrands;
    int m_ProjectedStrandsCount;
    GLuint m_VAO_RootProjectedStrands, m_VBO_RootProjectedStrands;

    glm::vec3 m_Root_Pos, m_Junction_Pos, m_Node_Tip1, m_Node_Tip2;
    glm::vec3 m_Junction_u_axis, m_Junction_v_axis;

    std::vector<glm::vec3> m_DebugGeneratedNodes; // 自動生成ノードの座標用
    std::vector<glm::vec3> m_DebugNormalNodes;    // 通常ノードの座標用

};