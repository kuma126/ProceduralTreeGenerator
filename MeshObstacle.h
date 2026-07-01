#pragma once
#include "Obstacle.h"
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cfloat>

// CGALのヘッダ群
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>


// CGALの型定義 (カーネルはdouble精度推奨)
typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point_3;
typedef K::Vector_3 Vector_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;


class MeshObstacle : public Obstacle {
private:
    Mesh m_Mesh;
    Tree m_Tree;
    // 内外判定用のクラス
    CGAL::Side_of_triangle_mesh<Mesh, K>* m_InsideTester = nullptr;

    glm::vec3 m_Scale;
    glm::vec3 m_Offset;

    glm::vec3 m_minBounds;
    glm::vec3 m_maxBounds;


public:
    // コンストラクタ: ファイルパス、スケール、位置オフセットを指定
    MeshObstacle(const std::string& filename, const glm::vec3& scale = glm::vec3(1.0f), const glm::vec3& offset = glm::vec3(0.0f))
        : m_Scale(scale), m_Offset(offset)
    {
        std::cout << "Loading mesh: " << filename << "..." << std::endl;

        // OBJファイルの読み込み
        if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(filename, m_Mesh)) {
            std::cerr << "ERROR: Failed to load mesh file: " << filename << std::endl;
            return;
        }

        // 初期化: バウンディングボックス計算用
        m_minBounds = glm::vec3(FLT_MAX);
        m_maxBounds = glm::vec3(-FLT_MAX);

        // メッシュの頂点を変換（スケール・移動）
        //    ※AABBツリー構築前に変形しておくのが効率的
        for (auto v : m_Mesh.vertices()) {
            Point_3& p = m_Mesh.point(v);
            // CGAL(double) -> GLM(float) -> 変換 -> CGAL(double)
            double x = p.x() * scale.x + offset.x;
            double y = p.y() * scale.y + offset.y;
            double z = p.z() * scale.z + offset.z;
            p = Point_3(x, y, z);

            //バウンディングボックスの更新
            glm::vec3 pos(x, y, z);
            m_minBounds = glm::min(m_minBounds, pos);
            m_maxBounds = glm::max(m_maxBounds, pos);
        }

        //すべての面を三角形に分割しておく
        CGAL::Polygon_mesh_processing::triangulate_faces(m_Mesh);

        // AABBツリーの構築 (高速化のため)
        m_Tree.insert(faces(m_Mesh).first, faces(m_Mesh).second, m_Mesh);
        m_Tree.build();

        // 内外判定器の初期化
        //    メッシュが閉じている必要がある
        m_InsideTester = new CGAL::Side_of_triangle_mesh<Mesh, K>(m_Tree);

        std::cout << "Mesh loaded successfully. Vertices: " << m_Mesh.number_of_vertices() << std::endl;
    }

    ~MeshObstacle() {
        if (m_InsideTester) delete m_InsideTester;
    }

    // インターフェースの実装
    void getDistanceAndNormal(const glm::vec3& p, float& outDist, glm::vec3& outNormal) const override {
        if (m_Tree.empty()) return;

        // GLM -> CGAL 座標変換
        Point_3 query(p.x, p.y, p.z);

        // ツリーを使ってメッシュ上の「最も近い点」を探す
        Point_3 closest = m_Tree.closest_point(query);

        // 距離の計算
        double sqDist = CGAL::squared_distance(query, closest);
        double dist = std::sqrt(sqDist);

        // 内外判定
        CGAL::Bounded_side side = m_InsideTester->operator()(query);

        if (side == CGAL::ON_BOUNDED_SIDE) {
            outDist = -(float)dist; // 内部なら負
        }
        else {
            outDist = (float)dist;  // 外部なら正
        }

        // 法線の計算 (クエリ点 - 最近点)
        Vector_3 diff = query - closest;

        // ゼロ除算対策
        if (dist > 0.00001) {
            // CGAL -> GLM 変換
            outNormal = glm::vec3((float)(diff.x() / dist), (float)(diff.y() / dist), (float)(diff.z() / dist));

            if (side == CGAL::ON_BOUNDED_SIDE) {
                outNormal = -outNormal; // 内部にいるときはベクトルを反転させて「外向き」にする
            }
        }
        else {
            outNormal = glm::vec3(0, 1, 0);
        }
    }


    // 描画用の頂点データを取得する関数
    void getTriangleVertices(std::vector<glm::vec3>& outVertices) {
        outVertices.clear();

        // メッシュの全「面(Face)」をループ
        for (auto f : m_Mesh.faces()) {
            // 三角形分割済みなので、各面は必ず3つの頂点を持つ
            // Face周りの頂点を取得するためのサーキュレータ
            auto v_start = m_Mesh.vertices_around_face(m_Mesh.halfedge(f)).begin();
            auto v_end = m_Mesh.vertices_around_face(m_Mesh.halfedge(f)).end();

            for (auto v_it = v_start; v_it != v_end; ++v_it) {
                // CGALのPoint_3を取得
                Point_3 p = m_Mesh.point(*v_it);

                // GLMのvec3に変換してリストに追加
                outVertices.push_back(glm::vec3((float)p.x(), (float)p.y(), (float)p.z()));
            }
        }
    }

    //境界ボックス情報の取得
    glm::vec3 getMinBounds() const { return m_minBounds; }
    glm::vec3 getMaxBounds() const { return m_maxBounds; }

    bool ComputeClosestPoint(
        const glm::vec3& p,
        glm::vec3& outClosest,
        float& outDistance
    ) const {
        glm::vec3 minB = getMinBounds();
        glm::vec3 maxB = getMaxBounds();

        // AABB への最近接点（clamp）
        outClosest.x = glm::clamp(p.x, minB.x, maxB.x);
        outClosest.y = glm::clamp(p.y, minB.y, maxB.y);
        outClosest.z = glm::clamp(p.z, minB.z, maxB.z);

        glm::vec3 diff = p - outClosest;
        outDistance = glm::length(diff);

        return true;
    }

   
  
};