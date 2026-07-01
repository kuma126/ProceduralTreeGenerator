#pragma once

#include <glm/glm.hpp>
#include <algorithm>

class Obstacle {
public:
	virtual ~Obstacle() = default;

	// îCà”ÇÃ3Dç¿ïW p Ç…Ç®ÇØÇÈ:
	// outDist   : ï\ñ Ç‹Ç≈ÇÃïÑçÜïtÇ´ãóó£ (ïâÇ»ÇÁì‡ïî)
	// outNormal : ç≈íZãóó£ï˚å¸ÇÃñ@ê¸ (ê≥ãKâªçœÇ›)
	virtual void getDistanceAndNormal(
		const glm::vec3& p,
		float& outDist,
		glm::vec3& outNormal) const = 0;

    float enabled = true;

    float SingleDistance(const glm::vec3& p) const {
        float d;
        glm::vec3 n;
        getDistanceAndNormal(p, d, n);
        return d;
    }
};


// SphereObstacle (ãÖëÃ)
class SphereObstacle : public Obstacle {
    glm::vec3 m_Center;
    float m_Radius;
public:
    SphereObstacle(const glm::vec3& c, float r) : m_Center(c), m_Radius(r) {}

    void getDistanceAndNormal(const glm::vec3& p, float& outDist, glm::vec3& outNormal) const override {
        glm::vec3 diff = p - m_Center;
        float distFromCenter = glm::length(diff);

        outDist = distFromCenter - m_Radius;

        if (distFromCenter > 0.0001f) {
            outNormal = diff / distFromCenter;
        }
        else {
            outNormal = glm::vec3(0, 1, 0); // íÜêSÇ…Ç¢ÇÈèÍçáÇÃà¿ëSçÙ
        }
    }
};


// InfiniteCylinderObstacle (ñ≥å¿â~íå - ä˘ë∂ìÆçÏÇÃçƒåª)
class InfiniteCylinderObstacle : public Obstacle {
    glm::vec3 m_CenterXZ; // Yé≤ñ≥å¿â~íåÇ∆âºíË (íÜêSÇÃX, ZÇÃÇ›égóp)
    float m_Radius;
public:
    InfiniteCylinderObstacle(const glm::vec3& c, float r) : m_CenterXZ(c), m_Radius(r) {}

    void getDistanceAndNormal(const glm::vec3& p, float& outDist, glm::vec3& outNormal) const override {
        // Yê¨ï™Çñ≥éãÇµÇƒXZïΩñ Ç≈ÇÃãóó£Çë™ÇÈ
        glm::vec3 p_xz = glm::vec3(p.x, 0.0f, p.z);
        glm::vec3 c_xz = glm::vec3(m_CenterXZ.x, 0.0f, m_CenterXZ.z);

        glm::vec3 diff = p_xz - c_xz;
        float distFromAxis = glm::length(diff);

        outDist = distFromAxis - m_Radius;

        if (distFromAxis > 0.0001f) {
            outNormal = diff / distFromAxis; // NormalÇ‡XZïΩñ è„
        }
        else {
            outNormal = glm::vec3(1, 0, 0);
        }
    }

    
};