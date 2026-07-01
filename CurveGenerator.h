#pragma once
#include <vector>
#include <glm/glm.hpp>

class CurveGenerator {
public:
    // B-Spline曲線を生成して vertices に格納する
    // segmentsPerSpan: 制御点間を何分割するか
    static void generateBSpline(
        const std::vector<glm::vec3>& controlPoints,
        std::vector<glm::vec3>& outVertices,
        int segmentsPerSpan = 10
    );

    // ファーガソン曲線（Catmull-Rom）の生成
    static void generateFerguson(
        const std::vector<glm::vec3>& controlPoints,
        std::vector<glm::vec3>& outVertices,
        int segmentsPerSpan = 10 // 分割数
    );

    // ファーガソン曲線の評価関数
    static glm::vec3 evalFerguson(
        const glm::vec3& p0, const glm::vec3& p1, // 始点・終点
        const glm::vec3& t0, const glm::vec3& t1, // 始点接線・終点接線
        float t
    );

private:
    // B-Splineの基底関数計算 (1点分)
    static glm::vec3 evalBSpline(
        const glm::vec3& p0, const glm::vec3& p1,
        const glm::vec3& p2, const glm::vec3& p3,
        float t
    );
};