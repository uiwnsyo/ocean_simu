#pragma once
/**
 * @file RigidBody.h
 * @brief 리지드 바디 + 충돌 형상 정의.
 *
 * 충돌 형상: Box, Sphere, Capsule, ConvexHull
 * 적분: 시뮬렉틱 오일러 (Symplectic Euler)
 * 좌표계: Y-up, Right-hand (Math.h 기준)
 */

#include "AGMath.h"
#include <vector>
#include <memory>
#include <cstdint>

namespace AG {

// ============================================================================
// AABB — Broad phase용 Axis-Aligned Bounding Box
// ============================================================================

struct AABB {
    float3 min = float3( 1e9f);
    float3 max = float3(-1e9f);

    /// @brief 두 AABB가 겹치는지 검사한다.
    bool Overlaps(const AABB& o) const {
        return (min.x <= o.max.x && max.x >= o.min.x) &&
               (min.y <= o.max.y && max.y >= o.min.y) &&
               (min.z <= o.max.z && max.z >= o.min.z);
    }

    /// @brief 두 AABB의 합집합.
    AABB Union(const AABB& o) const {
        return { glm::min(min, o.min), glm::max(max, o.max) };
    }

    float3 Center()   const { return (min + max) * 0.5f; }
    float3 HalfSize() const { return (max - min) * 0.5f; }

    /// @brief AABB 표면적 (BVH SAH 비용 계산용).
    float SurfaceArea() const {
        float3 d = max - min;
        return 2.0f * (d.x*d.y + d.y*d.z + d.z*d.x);
    }
};

// ============================================================================
// CollisionShape — 추상 기반
// ============================================================================

enum class ShapeType { Box, Sphere, Capsule, ConvexHull };

/**
 * @brief 충돌 형상 기반 클래스.
 *
 * 서브클래스는 GJK 지원 함수(Support)와 로컬 AABB를 제공해야 한다.
 */
struct CollisionShape {
    ShapeType type;
    explicit CollisionShape(ShapeType t) : type(t) {}
    virtual ~CollisionShape() = default;

    /**
     * @brief GJK/EPA 지원 함수 — 방향 d 에서 가장 먼 점 반환.
     * @param d  검색 방향 (월드 공간)
     * @param transform 형상의 월드 변환 행렬
     */
    virtual float3 Support(const float3& d, const float4x4& transform) const = 0;

    /// @brief 로컬 공간 AABB를 반환한다.
    virtual AABB   LocalAABB() const = 0;
};

// ---- Box ----

struct BoxShape final : CollisionShape {
    float3 halfExtents; ///< 각 축 방향 절반 크기

    explicit BoxShape(const float3& he)
        : CollisionShape(ShapeType::Box), halfExtents(he) {}

    float3 Support(const float3& d, const float4x4& transform) const override {
        // 로컬 방향으로 변환 후 부호 결정
        float4x4 invT = glm::inverse(transform);
        float3 localD = glm::normalize(float3(invT * float4(d, 0.0f)));
        float3 localP = float3(
            localD.x >= 0.0f ? halfExtents.x : -halfExtents.x,
            localD.y >= 0.0f ? halfExtents.y : -halfExtents.y,
            localD.z >= 0.0f ? halfExtents.z : -halfExtents.z);
        return float3(transform * float4(localP, 1.0f));
    }

    AABB LocalAABB() const override {
        return { -halfExtents, halfExtents };
    }
};

// ---- Sphere ----

struct SphereShape final : CollisionShape {
    float radius;

    explicit SphereShape(float r)
        : CollisionShape(ShapeType::Sphere), radius(r) {}

    float3 Support(const float3& d, const float4x4& transform) const override {
        float3 center = float3(transform[3]);
        return center + glm::normalize(d) * radius;
    }

    AABB LocalAABB() const override {
        return { float3(-radius), float3(radius) };
    }
};

// ---- Capsule ----

struct CapsuleShape final : CollisionShape {
    float radius;
    float halfHeight; ///< 실린더 절반 높이 (Y축 기준)

    CapsuleShape(float r, float hh)
        : CollisionShape(ShapeType::Capsule), radius(r), halfHeight(hh) {}

    float3 Support(const float3& d, const float4x4& transform) const override {
        float4x4 invT = glm::inverse(transform);
        float3 localD = float3(invT * float4(d, 0.0f));
        // 원통축 방향 선분에서 가장 먼 끝점 선택
        float3 tip = float3(0.0f, localD.y >= 0.0f ? halfHeight : -halfHeight, 0.0f);
        float3 localP = tip + glm::normalize(localD) * radius;
        return float3(transform * float4(localP, 1.0f));
    }

    AABB LocalAABB() const override {
        return { float3(-radius, -halfHeight-radius, -radius),
                 float3( radius,  halfHeight+radius,  radius) };
    }
};

// ---- Convex Hull ----

struct ConvexHullShape final : CollisionShape {
    std::vector<float3> vertices; ///< 로컬 공간 볼록 껍질 점

    explicit ConvexHullShape(std::vector<float3> verts)
        : CollisionShape(ShapeType::ConvexHull), vertices(std::move(verts)) {}

    float3 Support(const float3& d, const float4x4& transform) const override {
        float4x4 invT  = glm::inverse(transform);
        float3   localD = float3(invT * float4(d, 0.0f));
        float3   best  = vertices[0];
        float    bestDot = glm::dot(best, localD);
        for (const auto& v : vertices) {
            float dot = glm::dot(v, localD);
            if (dot > bestDot) { bestDot = dot; best = v; }
        }
        return float3(transform * float4(best, 1.0f));
    }

    AABB LocalAABB() const override {
        AABB aabb;
        for (const auto& v : vertices) {
            aabb.min = glm::min(aabb.min, v);
            aabb.max = glm::max(aabb.max, v);
        }
        return aabb;
    }
};

// ============================================================================
// RigidBody
// ============================================================================

using RigidBodyId = uint32_t;
static constexpr RigidBodyId INVALID_RIGIDBODY = 0xFFFFFFFFu;

/**
 * @brief 리지드 바디 단일 인스턴스.
 *
 * 위치·회전은 (m_position, m_orientation) 으로 유지되며,
 * 매 스텝마다 Symplectic Euler 적분을 수행한다.
 */
class RigidBody {
public:
    RigidBodyId id = INVALID_RIGIDBODY;

    // ---- Kinematic ----
    float3 position    = float3(0.0f);
    quat   orientation = quat(1, 0, 0, 0);
    float3 linearVel   = float3(0.0f);
    float3 angularVel  = float3(0.0f);   ///< 월드 공간 각속도 (rad/s)

    // ---- Mass Properties ----
    float  mass        = 1.0f;
    float  invMass     = 1.0f;           ///< 0 = static body
    float4x4 localInertiaTensorInv = float4x4(1.0f);

    // ---- Material ----
    float  restitution = 0.3f;          ///< 반발 계수 [0,1]
    float  friction    = 0.5f;          ///< 마찰 계수 (μ)

    // ---- Accumulated forces (cleared each step) ----
    float3 forceAccum  = float3(0.0f);
    float3 torqueAccum = float3(0.0f);

    // ---- Collision ----
    std::shared_ptr<CollisionShape> shape;
    bool isStatic = false;              ///< true = 질량 무한, 이동 안 함

    // ---- Flags ----
    bool isSleeping  = false;

    // ---- API ----

    /** @brief 력을 월드 공간 위치에 가한다. */
    void ApplyForce(const float3& force) { forceAccum += force; }

    /** @brief 위치에서의 힘 (토크 포함). */
    void ApplyForceAt(const float3& force, const float3& worldPoint) {
        forceAccum  += force;
        torqueAccum += glm::cross(worldPoint - position, force);
    }

    /** @brief 충격량(impulse)을 즉시 속도에 반영. */
    void ApplyImpulse(const float3& impulse, const float3& worldOffset) {
        if (isStatic) return;
        linearVel  += impulse * invMass;
        angularVel += WorldInertiaTensorInv() * glm::cross(worldOffset, impulse);
    }

    /** @brief 월드 공간 관성 텐서 역행렬. */
    float3x3 WorldInertiaTensorInv() const {
        float4x4 R = glm::mat4_cast(orientation);
        float4x4 worldI = R * localInertiaTensorInv * glm::transpose(R);
        return float3x3(worldI);
    }

    /** @brief 현재 변환 행렬 (TRS). */
    float4x4 GetTransform() const {
        return glm::translate(float4x4(1.0f), position)
             * glm::mat4_cast(orientation);
    }

    /** @brief 월드 AABB 계산. */
    AABB GetWorldAABB() const {
        if (!shape) return { position - float3(0.5f), position + float3(0.5f) };
        AABB local = shape->LocalAABB();
        float4x4 T = GetTransform();
        // 변환 행렬로 AABB 확대 (보수적)
        float3 center = float3(T * float4(local.Center(), 1.0f));
        float3 hs     = local.HalfSize();
        float3x3 absR = float3x3(
            glm::abs(float3(T[0])), glm::abs(float3(T[1])), glm::abs(float3(T[2])));
        float3 newHS  = absR * hs;
        return { center - newHS, center + newHS };
    }

    /** @brief 힘 누적을 초기화한다 (스텝 시작 시 호출). */
    void ClearForces() { forceAccum = float3(0.0f); torqueAccum = float3(0.0f); }

    /** @brief 관성 텐서의 역행렬을 질량과 형상에서 사전 계산한다. */
    void ComputeInertia() {
        if (isStatic) { invMass = 0.0f; localInertiaTensorInv = float4x4(0.0f); return; }
        invMass = 1.0f / mass;
        float3 he(0.5f);
        if (shape) he = shape->LocalAABB().HalfSize();
        float3 i = float3(
            (1.0f/12.0f) * mass * (4*he.y*he.y + 4*he.z*he.z),
            (1.0f/12.0f) * mass * (4*he.x*he.x + 4*he.z*he.z),
            (1.0f/12.0f) * mass * (4*he.x*he.x + 4*he.y*he.y));
        localInertiaTensorInv = float4x4(
            float4(1.0f/i.x, 0,0,0), float4(0,1.0f/i.y,0,0),
            float4(0,0,1.0f/i.z,0),  float4(0,0,0,1));
    }

private:
    using float3x3 = glm::mat3;
};

} // namespace AG
