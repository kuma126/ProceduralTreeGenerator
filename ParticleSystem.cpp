#include "ParticleSystem.h"
#include <iostream>
#include <algorithm>
#include <glm/gtx/string_cast.hpp>

ParticleSystem::ParticleSystem(float profileRadius, float particleRadius)
    : m_ProfileRadius(profileRadius), m_ParticleRadius(particleRadius)
{
}

ParticleSystem::~ParticleSystem() {}

void ParticleSystem::addParticle(const glm::vec2& position, int id) {
    PBDParticle p;
    p.position = position;
    p.prev_position = position;
    p.velocity = glm::vec2(0.0f, 0.0f);
    p.invMass = 1.0f;

    // 生成時の内外判定
    float dist = glm::length(position);
    p.initially_inside = (dist < (m_ProfileRadius - m_ParticleRadius));
    p.id = id;
    m_Particles.push_back(p);
}

void ParticleSystem::clearParticles() {
    m_Particles.clear();
}

// PBD メインループ (2D版の update() 相当)
void ParticleSystem::solve(int iterations) {
    // 予測ステップ 
    updatePhysics(DT);

    // 制約解決ループ
    for (int i = 0; i < iterations; ++i) {
        solveConstraints();
    }

    // 速度更新
    float maxVel = 0.0f; //デバッグ用
    float maxPos = 0.0f; //デバッグ用

    // 速度と位置の更新 (2D版の DAMPING_FACTOR 処理相当)
    for (auto& p : m_Particles) {
        if (p.invMass == 0.0f) continue;

        // 位置の変化分から速度を更新
        glm::vec2 newVelocity = (p.position - p.prev_position) / DT;

        // ダンピング
        p.velocity = newVelocity * (1.0f - DAMPING);

        // 安全のための速度制限 (2Dにはないが、3Dの爆発防止用保険)
        if (glm::length(p.velocity) > MAX_VELOCITY) {
            p.velocity = glm::normalize(p.velocity) * MAX_VELOCITY;
        }

       

        /// デバッグ監視コード ///
        float vLen = glm::length(p.velocity);
        float pLen = glm::length(p.position);
        if (vLen > maxVel) maxVel = vLen;
        if (pLen > maxPos) maxPos = pLen;

        // もし異常な速度ならログを出す
        if (vLen > 100.0f || std::isnan(vLen)) {
            std::cout << "[ALERT] Explosion Detected! ID:" << p.id
                << " Vel:" << vLen
                << " Pos:" << glm::to_string(p.position)
                << " Prev:" << glm::to_string(p.prev_position) << std::endl;
        }
    }

    // 全体の最大値を表示
    if (maxVel > 10.0f) {
        std::cout << "[INFO] Iteration Done. Max Vel: " << maxVel << " Max Pos: " << maxPos << std::endl;
    }
}


// 物理挙動 (Prediction Step)
void ParticleSystem::updatePhysics(float dt) {
    for (auto& p : m_Particles) {
        if (p.invMass == 0.0f) continue;

        // 仮位置の更新
        p.prev_position = p.position; // 現在位置を保存
        p.position += p.velocity * dt;
    }
}

// 制約解決
void ParticleSystem::solveConstraints() {
    const float ATTRACTION_STIFFNESS = 0.1f;
    const float CENTER_PULL = 0.02f;
    const float OBSTACLE_MARGIN  = 0.02f;

    float limitRadius = m_ProfileRadius - m_ParticleRadius;

    for (auto& p : m_Particles) {
        if (p.invMass == 0.0f) continue;

        bool hitObstacle = false;

        // =====================================================
        // [1] 障害物拘束（Hard Constraint）
        //     → 最近傍点へ直接スナップ
        // =====================================================
        if (m_pObstacle) {
            // 2D → 3D
            glm::vec3 pos3D =
                m_NodeOrigin
                + p.position.x * m_NodeU
                + p.position.y * m_NodeV;

            float dist3D;
            glm::vec3 normal3D;
            m_pObstacle->getDistanceAndNormal(pos3D, dist3D, normal3D);

            float threshold = m_ParticleRadius + OBSTACLE_MARGIN;

            if (dist3D < threshold) {
                hitObstacle = true;

                // --- 障害物表面の最近傍点（粒子半径分外側） ---
                // signed distance 前提
                glm::vec3 closest3D =
                    pos3D - normal3D * (dist3D - threshold);

                // --- 3D → 2D 差分として射影 ---
                glm::vec3 delta3D = closest3D - pos3D;

                glm::vec2 delta2D(
                    glm::dot(delta3D, m_NodeU),
                    glm::dot(delta3D, m_NodeV)
                );
                p.position += delta2D;
            }
        }

        // =====================================================
        // [2] プロファイル拘束（Soft Constraint）
        // =====================================================
        float distFromCenter = glm::length(p.position);

        if ((hitObstacle || !p.initially_inside) &&
            distFromCenter > limitRadius &&
            distFromCenter > 0.0001f) {

            float overlap = distFromCenter - limitRadius;

            glm::vec2 dirToCenter = -p.position / distFromCenter;

            glm::vec2 correction =
                dirToCenter * (overlap * ATTRACTION_STIFFNESS);

            p.position += correction;
        }

        // =====================================================
        // [3] 内部粒子の安定化（Stabilization）
        // =====================================================
        else if (p.initially_inside && !hitObstacle) {
            // 重心方向へごく弱く
            p.position += -p.position * CENTER_PULL;
        }
    }

    // =====================================================
    // [4] 粒子間衝突
    // =====================================================
    solveCollisions();
}

void ParticleSystem::solveCollisions() {
    float diameter = 2.0f * m_ParticleRadius;
    float diameterSq = diameter * diameter;

    // O(N^2) 総当たり (2D版と同ロジック)
    for (size_t i = 0; i < m_Particles.size(); ++i) {
        for (size_t j = i + 1; j < m_Particles.size(); ++j) {
            PBDParticle& p1 = m_Particles[i];
            PBDParticle& p2 = m_Particles[j];

            glm::vec2 dir = p1.position - p2.position;
            float distSq = glm::dot(dir, dir);

            if (distSq < diameterSq && distSq > 0.000001f) {
                float dist = std::sqrt(distSq);
                float penetration = diameter - dist;
                glm::vec2 normal = dir / dist;

                // 0.5ずつ押し出す (2D版: p1 -= response * 0.5, p2 += response * 0.5)
                glm::vec2 correction = normal * (penetration * 0.5f);

                p1.position += correction;
                p2.position -= correction;
            }
        }
    }
}