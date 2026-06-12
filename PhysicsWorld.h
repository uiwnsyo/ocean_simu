#pragma once
/**
 * @file PhysicsWorld.h
 * @brief 리지드 바디 물리 월드.
 *
 * BVH 브로드 페이즈 + SAT/GJK-EPA 나로우 페이즈 + PGS 솔버.
 * 고정 타임스텝 1/120초. PhysicsThread가 소유하며 렌더와 분리 실행된다.
 */

#include "RigidBody.h"
#include "BVH.h"
#include "CollisionDetection.h"
#include <unordered_map>
#include <vector>
#include <memory>

namespace AG {

// ============================================================================
// PGS Constraint (Contact)
// ============================================================================

/**
 * @brief PGS 솔버용 접촉 제약 조건.
 * 법선 충격량(λn)과 마찰 충격량(λt)을 저장하여 warm-starting에 사용한다.
 */
struct ContactConstraint {
    ContactManifold manifold;
    float  lambdaN[4] = {};  ///< 누적 법선 충격량 (warm start)
    float  lambdaT1[4] = {}; ///< 마찰 방향 1 누적 충격량
    float  lambdaT2[4] = {}; ///< 마찰 방향 2 누적 충격량
};

// ============================================================================
// PhysicsWorld
// ============================================================================

/**
 * @brief 물리 시뮬레이션 월드.
 *
 * Step() 를 고정 타임스텝(1/120 s)로 호출한다.
 * FluidCoupler가 buoyancy force를 Step() 내에서 주입한다.
 */
class PhysicsWorld {
public:
    // ------------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------------

    float3   gravity        = float3(0.0f, -9.81f, 0.0f);
    int      solverIterations = 10;       ///< PGS Gauss-Seidel 반복 횟수
    float    fixedDt        = 1.0f / 120.0f;
    float    sleepThreshold = 0.02f;      ///< 속도 임계값 이하 → sleep

    // ------------------------------------------------------------------
    // Body Management
    // ------------------------------------------------------------------

    /**
     * @brief 새 리지드 바디를 월드에 추가한다.
     * @return 부여된 RigidBodyId
     */
    RigidBodyId AddBody(std::shared_ptr<RigidBody> body);

    /**
     * @brief 리지드 바디를 월드에서 제거한다.
     */
    void RemoveBody(RigidBodyId id);

    /**
     * @brief ID로 리지드 바디를 참조한다.
     */
    RigidBody*  GetBody(RigidBodyId id);

    // ------------------------------------------------------------------
    // Simulation
    // ------------------------------------------------------------------

    /**
     * @brief 물리 스텝을 1회 진행한다.
     *
     * 순서:
     *  1. 중력·외력 적분 (예측 속도)
     *  2. BVH 업데이트 → 브로드 페이즈
     *  3. SAT/GJK-EPA 나로우 페이즈
     *  4. PGS 솔버 10 iteration
     *  5. 위치·회전 확정 적분
     *  6. Sleep 처리
     *
     * @note FluidCoupler::ApplyBuoyancy() 를 이 함수 전에 호출해야 한다.
     */
    void Step();

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    const std::vector<std::shared_ptr<RigidBody>>& GetBodies() const { return m_bodies; }
    const BVHTree&  GetBVH() const { return m_bvh; }

private:
    // ---- Integration ----
    void IntegrateForces(float dt);    ///< F=ma 속도 예측 (중력 포함)
    void IntegrateVelocities(float dt); ///< v → 위치·회전 확정

    // ---- Broad Phase ----
    void BroadPhase(std::vector<std::pair<RigidBodyId, RigidBodyId>>& pairs);

    // ---- Narrow Phase ----
    void NarrowPhase(const std::vector<std::pair<RigidBodyId, RigidBodyId>>& pairs,
                     std::vector<ContactConstraint>& constraints);

    // ---- PGS Solver ----
    /**
     * @brief Projected Gauss-Seidel 10-iteration 솔버.
     *
     * 각 접촉점에 대해:
     *  - λ = (target_velocity - v_rel·n) / (invMass_A + invMass_B + crossTerm)
     *  - clamp λ ≥ 0 (non-penetration)
     *  - 마찰: λt ∈ [-μλn, +μλn]
     */
    void SolveConstraints(std::vector<ContactConstraint>& constraints, float dt);

    // ---- Sleep ----
    void UpdateSleep();

    // ---- State ----
    std::vector<std::shared_ptr<RigidBody>>     m_bodies;
    std::unordered_map<RigidBodyId, int>         m_bodyIndexMap; ///< id → m_bodies 인덱스
    std::unordered_map<RigidBodyId, int>         m_leafMap;      ///< id → BVH leaf 인덱스
    BVHTree                                      m_bvh;
    RigidBodyId                                  m_nextId = 0;
};

} // namespace AG
