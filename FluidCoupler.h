#pragma once
/**
 * @file FluidCoupler.h
 * @brief 리지드-유체 커플링.
 *
 * 접근 순서 (SharedSimulationBuffer):
 *   [OceanSimulator::Update()] → FFT → WriteToSharedBuffer()
 *       ↓
 *   [FluidCoupler::ApplyBuoyancy()]  (리지드 솔버 Step() 전에 호출)
 *       ↓
 *   [PhysicsWorld::Step()]
 *       ↓
 *   [SharedSimulationBuffer::CommitWrite()]  → 렌더 스레드 ReadAcquire
 *
 * 부력 계산:
 *   - 리지드 AABB XZ 영역의 그리드 셀을 순회
 *   - 수면 높이 = SampleHeightAt()
 *   - 침수 부피 ≈ 셀 면적 × max(0, waterY - bodyBottom)
 *   - F_buoyancy = ρ_water × g × V_submerged (위 방향)
 *
 * 파문(Splash):
 *   - 리지드 바디가 수면에 닿을 때 바디 속도에 비례한
 *     사인파 교란을 인접 그리드 셀에 추가한다.
 */

#include "PhysicsWorld.h"
#include "OceanSimulator.h"

namespace AG {

// ============================================================================
// FluidCoupler
// ============================================================================

/**
 * @brief 리지드-유체 커플링 컴포넌트.
 */
class FluidCoupler {
public:
    // ------------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------------

    float waterDensity   = 1025.0f;  ///< 해수 밀도 (kg/m³)
    float splashStrength = 0.3f;     ///< 파문 교란 강도
    int   splashRadius   = 3;        ///< 파문 영향 셀 반경

    // ------------------------------------------------------------------
    // Coupling
    // ------------------------------------------------------------------

    /**
     * @brief 모든 리지드 바디에 부력을 적용한다.
     *
     * PhysicsWorld::Step() 호출 직전에 실행해야 한다.
     *
     * @param world   리지드 바디 월드
     * @param ocean   오션 시뮬레이터 (변위 샘플링)
     * @param gravity 현재 중력 가속도 (음수 Y, m/s²)
     */
    void ApplyBuoyancy(PhysicsWorld& world,
                       const OceanSimulator& ocean,
                       const float3& gravity);

    /**
     * @brief 리지드 바디의 수면 충돌을 감지하고 파문을 기록한다.
     *
     * SharedSimulationBuffer의 write 버퍼에 교란을 직접 추가한다.
     * (이 함수는 PhysicsWorld::Step() 후에 호출한다)
     *
     * @param world  리지드 바디 월드
     * @param ocean  오션 시뮬레이터
     */
    void ApplySplash(const PhysicsWorld& world,
                     const OceanSimulator& ocean);

private:
    /**
     * @brief 단일 리지드 바디의 침수 부피를 계산한다.
     *
     * AABB 그리드 셀 적분:
     *   V = Σ (cellArea × max(0, surfaceY - cellBottomY))
     *
     * @param body   대상 리지드 바디
     * @param ocean  오션 시뮬레이터
     * @return       침수 부피 (m³)
     */
    float ComputeSubmergedVolume(const RigidBody& body,
                                  const OceanSimulator& ocean) const;

    /**
     * @brief 수면 진입 감지 — 이전 프레임 상태와 비교.
     */
    bool IsEnteringWater(const RigidBody& body,
                          const OceanSimulator& ocean) const;

    // 이전 프레임 바디 수면 위/아래 상태 추적
    std::unordered_map<RigidBodyId, bool> m_prevSubmerged;
};

} // namespace AG
