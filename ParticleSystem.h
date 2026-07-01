#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "Obstacle.h"


// PBD用パーティクル構造体
struct PBDParticle {
    glm::vec2 position;
    glm::vec2 prev_position;
    glm::vec2 velocity;
    bool initially_inside;
    float invMass;
    int id;
};

struct Triangle {
    glm::vec3 v0, v1, v2;
    glm::vec3 normal;
};

class ParticleSystem {
public:
    ParticleSystem(float profileRadius, float particleRadius);
    ~ParticleSystem();

    void setProfileRadius(float radius) { m_ProfileRadius = radius; }
    void addParticle(const glm::vec2& position, int id);
    void clearParticles();
    const std::vector<PBDParticle>& getParticles() const { return m_Particles; }

    // 障害物をセット
    void setObstacle(Obstacle* obstacle) { m_pObstacle = obstacle; };

    // 3Dコンテキスト設定
    void setNodeContext(const glm::vec3& origin, const glm::vec3& u, const glm::vec3& v) {
        m_NodeOrigin = origin;
        m_NodeU = u;
        m_NodeV = v;
    }

    void solve(int iterations);


    void resetVelocities() {
        for (auto& p : m_Particles) {
            p.velocity = glm::vec2(0.0f, 0.0f);
            p.prev_position = p.position;
        }
    }

private:
    void updatePhysics(float dt);
    void solveConstraints(); 
    void solveCollisions();

   
private:
    float m_ProfileRadius;
    float m_ParticleRadius;
    std::vector<PBDParticle> m_Particles;

    Obstacle* m_pObstacle = nullptr;

    glm::vec3 m_NodeOrigin;
    glm::vec3 m_NodeU;
    glm::vec3 m_NodeV;

    const float DT = 0.002f;
    const float DAMPING = 0.02f;
    const float ATTRACTION_ACCEL = 40.0f;
    const float MAX_VELOCITY = 5.0f; 
};