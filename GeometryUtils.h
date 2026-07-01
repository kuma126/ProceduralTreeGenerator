#pragma once

#include <vector>
#include <algorithm>
#include <glm/glm.hpp>

// 外積（クロス積）の2D版
inline float cross_product(const glm::vec2& a, const glm::vec2& b, const glm::vec2& o) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// 凸包（Convex Hull）を計算する関数
// 入力: 2D点群, 出力: 外周の点群
inline std::vector<glm::vec2> getConvexHull(std::vector<glm::vec2>& points) {
    int n = points.size();
    if (n <= 2) return points; // 点が2個以下ならそのまま返す

    // X座標（同じならY座標）でソート
    std::sort(points.begin(), points.end(), [](const glm::vec2& a, const glm::vec2& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
        });

    std::vector<glm::vec2> hull;

    // 下側凸包 (Lower Hull) の構築
    for (int i = 0; i < n; ++i) {
        while (hull.size() >= 2 && cross_product(hull[hull.size() - 2], hull.back(), points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }

    // 上側凸包 (Upper Hull) の構築
    int lower_hull_size = hull.size();
    for (int i = n - 2; i >= 0; --i) {
        while (hull.size() > lower_hull_size && cross_product(hull[hull.size() - 2], hull.back(), points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }

    // 最後の点は始点と重複するので削除
    hull.pop_back();

    return hull;
}

void getOrthonormalBasis(const glm::vec3& tangent, glm::vec3& u_axis, glm::vec3& v_axis) {
    // 接線と平行にならないように、仮の上方向ベクトルを選ぶ
    glm::vec3 tempUp = glm::vec3(0.0f, 1.0f, 0.0f);
    if (glm::abs(glm::dot(tangent, tempUp)) > 0.9f) {
        tempUp = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    if (glm::abs(glm::dot(tangent, tempUp)) > 0.9f) {
        tempUp = glm::vec3(0.0f, 0.0f, 1.0f);
    }

    u_axis = glm::normalize(glm::cross(tempUp, tangent)); // 順序に注意 (右手系)
    v_axis = glm::normalize(glm::cross(tangent, u_axis));
}
