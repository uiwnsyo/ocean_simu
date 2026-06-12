#pragma once
/**
 * @file CollisionDetection.h
 * @brief Narrow Phase 충돌 검출 — SAT (Box), GJK-EPA (일반 Convex).
 */

#include "RigidBody.h"
#include <optional>
#include <array>

namespace AG {

// ============================================================================
// ContactPoint — 충돌 결과
// ============================================================================

/**
 * @brief 단일 충돌 접점 정보.
 */
struct ContactPoint {
    float3 pointA;      ///< 바디 A 위의 접촉점 (월드)
    float3 pointB;      ///< 바디 B 위의 접촉점 (월드)
    float3 normal;      ///< A→B 방향 충돌 법선
    float  penetration; ///< 침투 깊이 (> 0 = 겹침)
};

/**
 * @brief 두 바디 간 접촉 다양체 (최대 4점).
 */
struct ContactManifold {
    RigidBodyId bodyA = INVALID_RIGIDBODY;
    RigidBodyId bodyB = INVALID_RIGIDBODY;
    ContactPoint contacts[4];
    uint32_t     contactCount = 0;

    bool IsValid() const { return contactCount > 0; }
};

// ============================================================================
// SAT (Box vs Box)
// ============================================================================

/**
 * @brief OBB vs OBB — Separating Axis Theorem.
 *
 * 15개의 분리축(3+3+9)을 테스트한다.
 * 겹침이 있으면 ContactManifold를 반환한다.
 *
 * @param a   Box 형상 A
 * @param tA  A의 월드 변환 행렬
 * @param b   Box 형상 B
 * @param tB  B의 월드 변환 행렬
 */
std::optional<ContactManifold> TestBoxBox(
    const BoxShape& a, const float4x4& tA,
    const BoxShape& b, const float4x4& tB,
    RigidBodyId idA, RigidBodyId idB);

// ============================================================================
// GJK — Gilbert-Johnson-Keerthi
// ============================================================================

/**
 * @brief GJK 심플렉스 (최대 4점의 Minkowski Difference 서브셋).
 */
struct Simplex {
    std::array<float3, 4> pts;
    uint32_t size = 0;

    void Push(const float3& p) {
        pts[3] = pts[2]; pts[2] = pts[1]; pts[1] = pts[0];
        pts[0] = p;
        size = std::min(size + 1, 4u);
    }
    void Clear() { size = 0; }
};

/**
 * @brief GJK 거리/교차 테스트.
 *
 * @param shapeA, shapeB  충돌 형상
 * @param tA, tB          각각의 월드 변환
 * @return true if intersecting
 */
bool GJK_Intersects(const CollisionShape& shapeA, const float4x4& tA,
                    const CollisionShape& shapeB, const float4x4& tB,
                    Simplex& outSimplex);

// ============================================================================
// EPA — Expanding Polytope Algorithm
// ============================================================================

/**
 * @brief EPA로 침투 깊이와 충돌 법선을 계산한다.
 *
 * GJK에서 교차가 확인된 경우에만 호출한다.
 *
 * @param simplex  GJK에서 얻은 심플렉스 (교차 상태)
 * @param shapeA, shapeB  충돌 형상
 * @param tA, tB  각각의 월드 변환
 * @param[out] normal      충돌 법선 (A→B)
 * @param[out] penetration 침투 깊이 (m)
 * @return true if EPA converged
 */
bool EPA(const Simplex& simplex,
         const CollisionShape& shapeA, const float4x4& tA,
         const CollisionShape& shapeB, const float4x4& tB,
         float3& normal, float& penetration);

// ============================================================================
// NarrowPhase — 두 바디 간 충돌 검출 통합 진입점
// ============================================================================

/**
 * @brief 형상 타입에 따라 SAT 또는 GJK-EPA를 선택하여 충돌을 검출한다.
 *
 * - Box vs Box          → SAT
 * - Sphere vs anything  → GJK (구체 지원 함수가 빠름)
 * - 기타 Convex 조합    → GJK + EPA
 *
 * @return 충돌 있으면 ContactManifold, 없으면 nullopt
 */
std::optional<ContactManifold> DetectCollision(
    const RigidBody& a, const RigidBody& b);

} // namespace AG
