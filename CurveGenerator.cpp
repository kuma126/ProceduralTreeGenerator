#include "CurveGenerator.h"

void CurveGenerator::generateBSpline(
    const std::vector<glm::vec3>& controlPoints,
    std::vector<glm::vec3>& outVertices,
    int segmentsPerSpan)
{
    if (controlPoints.size() < 2) return;

    // B-Splineは端点を通らない性質があるため、始点と終点を重複させて
    // パディングすることで、端まで描画されるように調整する
    std::vector<glm::vec3> cp = controlPoints;

    // 始点を2回追加 (端点に寄せるため)
    cp.insert(cp.begin(), controlPoints.front());
    cp.insert(cp.begin(), controlPoints.front());

    // 終点を2回追加
    cp.push_back(controlPoints.back());
    cp.push_back(controlPoints.back());

    // 4点1セットでスライドしながら曲線を計算
    for (size_t i = 0; i < cp.size() - 3; ++i) {
        glm::vec3 p0 = cp[i];
        glm::vec3 p1 = cp[i + 1];
        glm::vec3 p2 = cp[i + 2];
        glm::vec3 p3 = cp[i + 3];

        glm::vec3 prevPos = evalBSpline(p0, p1, p2, p3, 0.0f);

        for (int j = 1; j <= segmentsPerSpan; ++j) {
            float t = (float)j / (float)segmentsPerSpan;
            glm::vec3 currentPos = evalBSpline(p0, p1, p2, p3, t);

            // GL_LINES 用に [始点, 終点] のペアで格納
            outVertices.push_back(prevPos);
            outVertices.push_back(currentPos);

            prevPos = currentPos;
        }
    }
}

glm::vec3 CurveGenerator::evalBSpline(
    const glm::vec3& p0, const glm::vec3& p1,
    const glm::vec3& p2, const glm::vec3& p3,
    float t)
{
    // Uniform Cubic B-Spline Basis Functions
    // 論文で採用されている滑らかな曲線近似
    float t2 = t * t;
    float t3 = t2 * t;

    // 係数計算: (1/6) * Matrix * [1 t t^2 t^3]
    float b0 = (1.0f - 3.0f * t + 3.0f * t2 - t3) / 6.0f;
    float b1 = (4.0f - 6.0f * t2 + 3.0f * t3) / 6.0f;
    float b2 = (1.0f + 3.0f * t + 3.0f * t2 - 3.0f * t3) / 6.0f;
    float b3 = t3 / 6.0f;

    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}


void CurveGenerator::generateFerguson(
    const std::vector<glm::vec3>& controlPoints,
    std::vector<glm::vec3>& outVertices,
    int segmentsPerSpan)
{
    // 点が2つ未満なら線は引けない
    if (controlPoints.size() < 2) return;

    // 各区間ごとに計算 (点i から 点i+1 へ)
    for (size_t i = 0; i < controlPoints.size() - 1; ++i) {

        glm::vec3 p0 = controlPoints[i];     // 始点
        glm::vec3 p1 = controlPoints[i + 1]; // 終点

        // --- 接線ベクトル(Tangent)の計算 ---
        // Catmull-Romスプラインのロジックを使用: T = (P_next - P_prev) / 2
        glm::vec3 t0, t1;

        // 始点の接線 t0
        if (i == 0) {
            // 最初の点は前の点がないので、単方向差分 (p1 - p0)
            t0 = p1 - p0;
        }
        else {
            // 中間点は前後から計算 (p_next - p_prev) * 0.5
            t0 = (controlPoints[i + 1] - controlPoints[i - 1]) * 0.5f;
        }

        // 終点の接線 t1
        if (i == controlPoints.size() - 2) {
            // 最後の点は次の点がないので、単方向差分
            t1 = p1 - p0;
        }
        else {
            t1 = (controlPoints[i + 2] - controlPoints[i]) * 0.5f;
        }

        // --- 補間ループ ---
        glm::vec3 prevPos = evalFerguson(p0, p1, t0, t1, 0.0f);

        for (int j = 1; j <= segmentsPerSpan; ++j) {
            float t = (float)j / (float)segmentsPerSpan;
            glm::vec3 currentPos = evalFerguson(p0, p1, t0, t1, t);

            // GL_LINES 用にペアで格納
            outVertices.push_back(prevPos);
            outVertices.push_back(currentPos);

            prevPos = currentPos;
        }
    }
}

glm::vec3 CurveGenerator::evalFerguson(
    const glm::vec3& p0, const glm::vec3& p1,
    const glm::vec3& tan0, const glm::vec3& tan1,
    float t)
{
    // ファーガソン曲線（Cubic Hermite Spline）の基底関数
    // P(t) = (2t^3 - 3t^2 + 1)P0 + (t^3 - 2t^2 + t)T0 + (-2t^3 + 3t^2)P1 + (t^3 - t^2)T1

    float t2 = t * t;
    float t3 = t2 * t;

    float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;      // P0の係数
    float h10 = t3 - 2.0f * t2 + t;                // T0の係数
    float h01 = -2.0f * t3 + 3.0f * t2;            // P1の係数
    float h11 = t3 - t2;                           // T1の係数

    return h00 * p0 + h10 * tan0 + h01 * p1 + h11 * tan1;
}